/*
 * rocket.c - low-level terminal rocket launch simulator.
 *
 * Build:
 *   gcc -O2 -std=c99 -Wall -Wextra rocket.c -lm -o rocket
 *
 * Controls while flying:
 *   w/s throttle  x cutoff  r max throttle  p pause  q abort
 */

#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define MIN_W 80
#define MIN_H 24
#define TRAIL 4096
#define PI 3.1415926535897932384626433832795
#define G0 9.80665
#define EARTH_R 6371000.0

#define ABSI(a) ((a) < 0 ? -(a) : (a))
#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define LEN(a) ((int)(sizeof(a) / sizeof((a)[0])))
typedef struct {
    int w, h;
    char *now;
    char *old;
} Frame;

typedef struct {
    double t, x, y, vx, vy;
    double fuel, dry, throttle, target, wind0;
    double max_q, max_g, max_alt, max_speed;
    double trail_x[TRAIL], trail_y[TRAIL];
    unsigned trail_n, seed;
    int done, success, crash, landed, paused, aborted;
} Sim;

typedef struct {
    double cam_x, cam_y, sx, sy;
} Camera;

static Frame fb;
static struct termios old_tty;
static int old_flags = -1;
static int raw_tty = 0;
static int term_live = 0;
static int resized = 1;

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void sleep_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    while (nanosleep(&ts, &ts) && errno == EINTR) {
    }
}

static uint32_t mix32(uint32_t x)
{
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

static uint32_t cell_hash(int x, int y, uint32_t seed)
{
    return mix32((uint32_t)x * 0x9e3779b1U ^ (uint32_t)y * 0x85ebca77U ^ seed);
}

static double unit_noise(int x, int y, uint32_t seed)
{
    return (double)(cell_hash(x, y, seed) & 0xffffU) / 65535.0;
}

static double smoothstep(double a, double b, double x)
{
    double t = CLAMP((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

static void die_clean(int sig)
{
    (void)sig;
    if (term_live) {
        fputs("\033[0m\033[?25h\033[?1049l", stdout);
        fflush(stdout);
    }
    _exit(128 + sig);
}

static void term_begin(void)
{
    struct termios t;

    if (!isatty(STDOUT_FILENO)) {
        return;
    }

    if (tcgetattr(STDIN_FILENO, &old_tty) == 0) {
        t = old_tty;
        t.c_lflag &= (tcflag_t)~(ICANON | ECHO);
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &t) == 0) {
            raw_tty = 1;
        }
    }
    old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (old_flags >= 0) {
        (void)fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);
    }

    fputs("\033[?1049h\033[?25l\033[2J", stdout);
    fflush(stdout);
    term_live = 1;
}

static void term_end(void)
{
    if (!term_live) {
        return;
    }
    if (raw_tty) {
        (void)tcsetattr(STDIN_FILENO, TCSANOW, &old_tty);
        raw_tty = 0;
    }
    if (old_flags >= 0) {
        (void)fcntl(STDIN_FILENO, F_SETFL, old_flags);
        old_flags = -1;
    }
    fputs("\033[0m\033[?25h\033[?1049l", stdout);
    fflush(stdout);
    term_live = 0;
}

static int term_size(int *w, int *h)
{
    struct winsize ws;

    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col && ws.ws_row) {
        *w = ws.ws_col;
        *h = ws.ws_row;
        return 1;
    }

    *w = 100;
    *h = 34;
    return 1;
}

static int get_key(void)
{
    unsigned char c;
    return read(STDIN_FILENO, &c, 1) == 1 ? (int)c : 0;
}

static void frame_free(void)
{
    free(fb.now);
    free(fb.old);
    fb.now = NULL;
    fb.old = NULL;
    fb.w = fb.h = 0;
}

static int frame_make(int w, int h)
{
    size_t n;

    if (w < MIN_W || h < MIN_H) {
        return 0;
    }
    if (w == fb.w && h == fb.h && fb.now && fb.old) {
        return 1;
    }

    frame_free();
    fb.w = w;
    fb.h = h;
    n = (size_t)w * (size_t)h;
    fb.now = (char *)malloc(n);
    fb.old = (char *)malloc(n);
    if (!fb.now || !fb.old) {
        frame_free();
        return 0;
    }
    memset(fb.now, ' ', n);
    memset(fb.old, 0, n);
    resized = 1;
    return 1;
}

static void frame_clear(char c)
{
    memset(fb.now, c, (size_t)fb.w * (size_t)fb.h);
}

static void pix(int x, int y, char c)
{
    if ((unsigned)x < (unsigned)fb.w && (unsigned)y < (unsigned)fb.h) {
        fb.now[y * fb.w + x] = c;
    }
}

static char at(int x, int y)
{
    if ((unsigned)x >= (unsigned)fb.w || (unsigned)y >= (unsigned)fb.h) {
        return ' ';
    }
    return fb.now[y * fb.w + x];
}

static void text(int x, int y, const char *s)
{
    while (*s) {
        pix(x++, y, *s++);
    }
}

static void fmt(int x, int y, const char *f, ...)
{
    char b[256];
    va_list ap;

    va_start(ap, f);
    vsnprintf(b, sizeof b, f, ap);
    va_end(ap);
    text(x, y, b);
}

static void line(int x0, int y0, int x1, int y1, char c)
{
    int dx = ABSI(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -ABSI(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int e = dx + dy;

    for (;;) {
        int e2;
        pix(x0, y0, c);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        e2 = e << 1;
        if (e2 >= dy) {
            e += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            e += dx;
            y0 += sy;
        }
    }
}

static void hline(int x0, int x1, int y, char c)
{
    int x;
    if (y < 0 || y >= fb.h) {
        return;
    }
    if (x0 > x1) {
        int t = x0;
        x0 = x1;
        x1 = t;
    }
    x0 = CLAMP(x0, 0, fb.w - 1);
    x1 = CLAMP(x1, 0, fb.w - 1);
    for (x = x0; x <= x1; ++x) {
        pix(x, y, c);
    }
}

static void clear_box(int x0, int y0, int x1, int y1)
{
    int y;
    if (x0 > x1) {
        int t = x0;
        x0 = x1;
        x1 = t;
    }
    if (y0 > y1) {
        int t = y0;
        y0 = y1;
        y1 = t;
    }
    y0 = CLAMP(y0, 0, fb.h - 1);
    y1 = CLAMP(y1, 0, fb.h - 1);
    for (y = y0; y <= y1; ++y) {
        hline(x0, x1, y, ' ');
    }
}

static void vline(int x, int y0, int y1, char c)
{
    int y;
    if (x < 0 || x >= fb.w) {
        return;
    }
    if (y0 > y1) {
        int t = y0;
        y0 = y1;
        y1 = t;
    }
    y0 = CLAMP(y0, 0, fb.h - 1);
    y1 = CLAMP(y1, 0, fb.h - 1);
    for (y = y0; y <= y1; ++y) {
        pix(x, y, c);
    }
}

static void text_centered(int cx, int y, const char *s)
{
    text(cx - (int)strlen(s) / 2, y, s);
}

static void frame_diff(void)
{
    int y, x;

    if (resized) {
        fputs("\033[2J", stdout);
        memset(fb.old, 0, (size_t)fb.w * (size_t)fb.h);
        resized = 0;
    }

    for (y = 0; y < fb.h; ++y) {
        x = 0;
        while (x < fb.w) {
            int a, b;
            while (x < fb.w && fb.now[y * fb.w + x] == fb.old[y * fb.w + x]) {
                ++x;
            }
            if (x >= fb.w) {
                break;
            }
            a = x;
            while (x < fb.w && fb.now[y * fb.w + x] != fb.old[y * fb.w + x]) {
                ++x;
            }
            b = x;
            printf("\033[%d;%dH", y + 1, a + 1);
            fwrite(fb.now + y * fb.w + a, 1, (size_t)(b - a), stdout);
            memcpy(fb.old + y * fb.w + a, fb.now + y * fb.w + a, (size_t)(b - a));
        }
    }
    fflush(stdout);
}

static double terrain_raw(double x)
{
    return 42.0 * sin(x * 0.00135 + 0.7) +
           18.0 * sin(x * 0.00490 - 1.1) +
            6.0 * sin(x * 0.01700 + 2.3);
}

static double terrain(double x)
{
    double r = terrain_raw(x);
    double ax = fabs(x);

    if (ax < 190.0) {
        return 0.0;
    }
    if (ax < 330.0) {
        double k = smoothstep(190.0, 330.0, ax);
        return r * k;
    }
    return r;
}

static Camera camera_for(const Sim *s)
{
    Camera c;
    double low_anchor, high_anchor, blend, anchor;
    c.sy = CLAMP(fmax(7.0, (s->y + 420.0) / (double)(fb.h / 2 + 8)), 7.0, 15000.0);
    c.sx = c.sy * 2.15;
    low_anchor = CLAMP((double)fb.h * 0.24, 5.5, 9.0);
    high_anchor = (double)fb.h * 0.53;
    blend = smoothstep(500.0, 2200.0, s->y);
    anchor = low_anchor * (1.0 - blend) + high_anchor * blend;
    c.cam_x = s->x - c.sx * (double)fb.w * 0.47;
    c.cam_y = s->y - c.sy * anchor;
    return c;
}

static void map_world(const Camera *c, double wx, double wy, int *x, int *y)
{
    *x = (int)floor((wx - c->cam_x) / c->sx + 0.5);
    *y = fb.h - 1 - (int)floor((wy - c->cam_y) / c->sy + 0.5);
}

static void world_line(const Camera *c, double x0, double y0, double x1, double y1, char ch)
{
    int ax, ay, bx, by;
    map_world(c, x0, y0, &ax, &ay);
    map_world(c, x1, y1, &bx, &by);
    line(ax, ay, bx, by, ch);
}

static void world_text(const Camera *c, double wx, double wy, const char *s)
{
    int x, y;
    map_world(c, wx, wy, &x, &y);
    text(x, y, s);
}

static void draw_plume(int cx, int base_y, double throttle, int height, int max_span, unsigned tick)
{
    int y, x;
    char core = throttle > 0.78 ? '#' : throttle > 0.45 ? '*' : '+';

    if (throttle <= 0.01 || height <= 0) {
        return;
    }

    for (y = 0; y < height; ++y) {
        double k = (double)y / (double)(height > 1 ? height - 1 : 1);
        int span = max_span <= 0 ? 0 : (int)(throttle * (double)max_span * (0.35 + k) + 0.5);
        for (x = -span; x <= span; ++x) {
            uint32_t h = mix32(tick + (uint32_t)(y * 911 + (x + 64) * 131));
            char c = span == 0 ? core : (ABSI(x) <= span / 3 ? core : (k < 0.55 ? '*' : '.'));
            if (span == 0 || (h & 7U) != 0U || ABSI(x) <= 1) {
                pix(cx + x, base_y + y, c);
            }
        }
    }
}

static void draw_rocket(int cx, int cy, double throttle, double sy, unsigned tick)
{
    static const char *big[] = {
        "/^\\",
        "/...\\",
        "/.....\\",
        "/=======\\",
        "|...|...|",
        "|..[o]..|",
        "|...|...|",
        "|---+---|",
        "|...|...|",
        "|=======|",
        "/|  |  |\\",
        "/_|__|__|_\\"
    };
    static const char *mid[] = {
        "/^\\",
        "/===\\",
        "|.O.|",
        "|.|.|",
        "|===|",
        "/|_|\\"
    };
    static const char *small[] = {
        " ^ ",
        "/|\\",
        " | "
    };
    static const char *tiny[] = {
        "^",
        "|"
    };
    const char **rows;
    int n, top, i;

    if (sy < 35.0) {
        rows = big;
        n = LEN(big);
        top = cy - n;
        draw_plume(cx, top + n, throttle, 10, 4, tick);
        vline(cx - 2, top + n - 2, top + n, '|');
        vline(cx + 2, top + n - 2, top + n, '|');
    } else if (sy < 180.0) {
        rows = mid;
        n = LEN(mid);
        top = cy - n;
        draw_plume(cx, top + n, throttle, 5, 2, tick);
        vline(cx, top + n - 1, top + n, '|');
    } else if (sy < 900.0) {
        rows = small;
        n = LEN(small);
        top = cy - n;
        draw_plume(cx, top + n, throttle, 2, 0, tick);
    } else {
        rows = tiny;
        n = LEN(tiny);
        top = cy - n;
        draw_plume(cx, top + n, throttle, 1, 0, tick);
    }

    for (i = 0; i < n; ++i) {
        text_centered(cx, top + i, rows[i]);
    }
}

static void draw_stars(const Camera *c)
{
    int x, y;

    for (y = 3; y < fb.h; ++y) {
        double wy = c->cam_y + (double)(fb.h - 1 - y) * c->sy;
        for (x = 0; x < fb.w; ++x) {
            double wx = c->cam_x + (double)x * c->sx;
            uint32_t h = cell_hash((int)floor(wx / 5200.0), (int)floor(wy / 4200.0), 0x5a5a5eedU);
            if ((h & 2047U) > 2034U) {
                pix(x, y, (h & 8192U) ? '+' : '.');
            }
        }
    }
}

static void draw_cloud_deck(const Camera *c, double lo, double hi, uint32_t seed)
{
    int x, y;

    for (y = 3; y < fb.h; ++y) {
        double wy = c->cam_y + (double)(fb.h - 1 - y) * c->sy;
        double band = (wy - lo) / (hi - lo);
        if (band < 0.0 || band > 1.0) {
            continue;
        }
        for (x = 0; x < fb.w; ++x) {
            double wx = c->cam_x + (double)x * c->sx;
            double swell = 0.64 * sin(wx * 0.0010 + (double)(seed & 31U)) +
                           0.30 * sin(wx * 0.0034 - band * 5.0);
            double lump = unit_noise((int)floor(wx / 860.0), (int)(lo / 100.0), seed);
            double edge = fabs(band - 0.5) * 0.62;
            if (swell + lump * 0.74 > 0.64 + edge) {
                char ch = band > 0.72 ? '_' : band < 0.20 ? '`' : '.';
                pix(x, y, ch);
            }
        }
    }
}

static void draw_cloud_ocean(const Sim *s)
{
    int x, x0, y, row;
    int base;
    double k, arch;

    if (s->y < 5200.0 || s->y > 36000.0) {
        return;
    }

    k = CLAMP((s->y - 5200.0) / 30800.0, 0.0, 1.0);
    base = fb.h - 5 - (int)(k * (double)fb.h * 0.26);
    base = CLAMP(base, fb.h / 2, fb.h - 4);
    arch = 1.5 + k * (double)fb.h * 0.16;

    for (x = 0; x < fb.w; ++x) {
        double dx = ((double)x - (double)fb.w * 0.5) / ((double)fb.w * 0.5);
        y = base + (int)(dx * dx * arch + 0.5);
        if (y < 3 || y >= fb.h) {
            continue;
        }
        if ((x & 3) == 0) {
            pix(x, y - 1, '.');
        }
        pix(x, y, '_');
    }

    for (x0 = -24 - ((int)(s->t * 2.0) % 24); x0 < fb.w + 24; x0 += 24) {
        double dx = ((double)(x0 + 7) - (double)fb.w * 0.5) / ((double)fb.w * 0.5);
        int cy = base + (int)(dx * dx * arch + 0.5) - 3 + (int)(2.0 * sin((double)x0 * 0.19 + s->t * 0.07));
        text(x0, cy,     "    .--.    ");
        text(x0, cy + 1, " .-(    )-. ");
        text(x0, cy + 2, "(___.__)___)");
    }

    for (row = base + 4; row < fb.h - 2; row += 4) {
        for (x = 0; x < fb.w; ++x) {
            double lane = sin((double)x * 0.055 + (double)row * 0.31 + s->t * 0.05);
            if (lane > 0.72) {
                pix(x, row, '_');
            } else if (lane > 0.56 && (x & 5) == 0) {
                pix(x, row, '.');
            }
        }
    }
}

static void draw_earth_limb(const Sim *s)
{
    int x, y, row;
    int base;
    double k, arch;

    if (s->y < 24000.0) {
        return;
    }

    k = CLAMP((s->y - 24000.0) / 105000.0, 0.0, 1.0);
    base = fb.h - 3 - (int)(k * (double)fb.h * 0.43);
    base = CLAMP(base, fb.h / 2, fb.h - 3);
    arch = (double)fb.h * (0.10 + 0.10 * k);

    for (x = 0; x < fb.w; ++x) {
        double dx = ((double)x - (double)fb.w * 0.5) / ((double)fb.w * 0.5);
        y = base + (int)(dx * dx * arch + 0.5);
        if (y < 4 || y >= fb.h) {
            continue;
        }

        pix(x, y - 3, '`');
        pix(x, y - 2, '.');
        pix(x, y - 1, '-');
        pix(x, y, '=');

        for (row = y + 1; row < fb.h; ++row) {
            double m = sin((double)x * 0.080 + (double)row * 0.115) +
                       0.65 * sin((double)x * 0.031 - (double)row * 0.071 + 1.7);
            double cloud = sin((double)x * 0.19 + (double)row * 0.07 + 2.0);
            char fill;

            if (row - y < 4 && cloud > 0.30) {
                fill = '.';
            } else if (m > 0.88) {
                fill = ',';
            } else if (m > 0.38) {
                fill = (row + x / 5) & 1 ? ',' : '.';
            } else {
                fill = (row + x / 9) % 7 == 0 ? '-' : '~';
            }
            pix(x, row, fill);
        }
    }
}

static void draw_sky(const Camera *c, const Sim *s)
{
    frame_clear(' ');

    if (s->y > 21000.0 || c->cam_y > 16000.0) {
        draw_stars(c);
    }

    if (c->cam_y < 13000.0 && s->y < 9000.0) {
        draw_cloud_deck(c, 1500.0, 2600.0, 0xc10d5101U);
        draw_cloud_deck(c, 6200.0, 7600.0, 0xc10d5202U);
    }

    draw_cloud_ocean(s);
    draw_earth_limb(s);

    if (s->y < 30000.0 && fb.w > 70 && fb.h > 20) {
        int mx = fb.w - 9, my = 5;
        text(mx - 2, my - 1, "\\ | /");
        text(mx - 2, my,   "- @ -");
        text(mx - 2, my + 1, "/ | \\");
    }
}

static void draw_distant_ridges(const Camera *c)
{
    int x, y, row;

    if (c->cam_y > 2600.0 || c->sy > 120.0) {
        return;
    }

    for (x = 0; x < fb.w; ++x) {
        double wx = c->cam_x + (double)x * c->sx;
        double wy = 36.0 + 4.0 * sin(wx * 0.0031) + 2.0 * sin(wx * 0.0097 + 1.4);
        int sx;
        map_world(c, wx, wy, &sx, &y);
        if (sx < 0 || sx >= fb.w || y < 5 || y >= fb.h - 5) {
            continue;
        }
        pix(sx, y, (sx & 5) == 0 ? '~' : '-');
        for (row = y + 1; row < y + 4 && row < fb.h; ++row) {
            if (((sx + row * 3) & 7) == 0) {
                pix(sx, row, row == y + 1 ? '`' : '.');
            }
        }
    }
}

static void draw_terrain(const Camera *c)
{
    int col, x, y, row;

    for (col = 0; col < fb.w; ++col) {
        double wx = c->cam_x + (double)col * c->sx;
        double wy = terrain(wx);
        double slope = terrain(wx + c->sx) - terrain(wx - c->sx);
        char crown = c->sy < 120.0 && slope > c->sy * 0.38 ? '/' :
                     c->sy < 120.0 && slope < -c->sy * 0.38 ? '\\' : '_';
        map_world(c, wx, wy, &x, &y);

        if (y < 0) {
            for (row = 3; row < fb.h; ++row) {
                pix(x, row, row % 5 == 0 ? '=' : ':');
            }
            continue;
        }
        if (y >= fb.h) {
            continue;
        }

        pix(x, y, c->sy < 55.0 ? (col % 10 < 5 ? '-' : '~') : crown);
        for (row = y + 1; row < fb.h; ++row) {
            int d = row - y;
            if (row >= fb.h - 2) {
                pix(x, row, '=');
            } else if (d == 1) {
                pix(x, row, '.');
            } else if (d == 2 && ((col + row) & 1) == 0) {
                pix(x, row, '.');
            } else if (((col * 5 + row * 3) % 29) == 0) {
                pix(x, row, ':');
            }
        }
    }
}

static void draw_close_launch_complex(const Camera *c, const Sim *s)
{
    int pad_x, ground_y;
    int deck, top, tower_x, y;
    double gy = terrain(0.0);

    map_world(c, 0.0, gy, &pad_x, &ground_y);
    if (ground_y < 9 || ground_y >= fb.h || pad_x < -30 || pad_x > fb.w + 30) {
        return;
    }

    deck = ground_y - 1;
    top = deck - 12;
    tower_x = pad_x - 16;

    hline(pad_x - 24, pad_x + 25, deck, '=');
    hline(pad_x - 20, pad_x + 21, deck + 1, '#');
    hline(pad_x - 9, pad_x + 9, deck + 2, '_');
    text(pad_x - 8, deck + 3, "\\______/ ");

    vline(tower_x, top, deck, '|');
    vline(tower_x + 4, top + 1, deck, '|');
    for (y = top + 1; y <= deck - 2; y += 2) {
        line(tower_x, y, tower_x + 4, y + 2, '\\');
        line(tower_x + 4, y, tower_x, y + 2, '/');
        hline(tower_x, tower_x + 4, y, '-');
    }

    hline(tower_x + 4, pad_x - 7, top + 4, '-');
    hline(tower_x + 4, pad_x - 7, top + 8, '-');
    line(tower_x + 5, top + 5, pad_x - 8, top + 7, '/');
    line(tower_x + 5, top + 9, pad_x - 8, top + 10, '/');
    pix(pad_x - 6, top + 4, ']');
    pix(pad_x - 6, top + 8, ']');

    vline(pad_x + 20, deck - 5, deck, '|');
    vline(pad_x + 28, deck - 5, deck, '|');
    hline(pad_x + 20, pad_x + 28, deck - 5, '_');
    pix(pad_x + 19, deck - 4, '/');
    hline(pad_x + 20, pad_x + 28, deck - 4, '-');
    pix(pad_x + 29, deck - 4, '\\');
    pix(pad_x + 19, deck - 3, '\\');
    hline(pad_x + 20, pad_x + 28, deck - 3, '_');
    pix(pad_x + 29, deck - 3, '/');
    text(pad_x + 21, deck - 4, "LOX");

    if (s->t < 0.0) {
        int ph = (int)(-s->t * 7.0) & 3;
        text(pad_x - 30, deck - 2, ph & 1 ? "~~  ~~" : "  ~~  ");
        text(pad_x + 11, deck - 1, ph & 2 ? " .~~" : "~~. ");
    }
}

static void draw_pad(const Camera *c, const Sim *s)
{
    int x0, y0, x1, y1, i;
    double gy = terrain(0.0);

    if (c->sy < 55.0) {
        draw_close_launch_complex(c, s);
        return;
    }

    if (c->sy > 120.0) {
        world_line(c, -190.0, gy + 1.0, 190.0, gy + 1.0, '=');
        world_text(c, -35.0, gy + 17.0, "LC-39");
        return;
    }

    map_world(c, -250.0, gy, &x0, &y0);
    map_world(c, 250.0, gy, &x1, &y1);
    hline(x0, x1, y0, '=');
    hline(x0 + 5, x1 - 5, y0 + 1, '#');
    hline(x0 + 13, x1 - 13, y0 + 2, '#');

    world_line(c, -98.0, gy, -98.0, gy + 126.0, '|');
    world_line(c, -80.0, gy, -80.0, gy + 111.0, '|');
    world_line(c, -112.0, gy + 126.0, -66.0, gy + 126.0, '-');
    world_line(c, -89.0, gy + 126.0, -89.0, gy + 154.0, '|');
    world_line(c, -105.0, gy + 111.0, -72.0, gy + 144.0, '/');
    world_line(c, -74.0, gy + 111.0, -107.0, gy + 144.0, '\\');

    for (i = 10; i < 116; i += 13) {
        world_line(c, -98.0, gy + i, -80.0, gy + i + 8.0, '/');
        world_line(c, -80.0, gy + i, -98.0, gy + i + 8.0, '\\');
        world_line(c, -101.0, gy + i, -76.0, gy + i, '-');
    }

    world_line(c, -80.0, gy + 84.0, -21.0, gy + 84.0, '=');
    world_line(c, -80.0, gy + 58.0, -31.0, gy + 58.0, '-');
    world_line(c, -80.0, gy + 33.0, -43.0, gy + 33.0, '-');
    world_line(c, -68.0, gy + 84.0, -47.0, gy + 66.0, '/');
    world_line(c, -69.0, gy + 58.0, -50.0, gy + 43.0, '/');

    world_line(c, 26.0, gy, 26.0, gy + 23.0, '|');
    world_line(c, -26.0, gy, -26.0, gy + 23.0, '|');
    world_line(c, -44.0, gy + 23.0, 44.0, gy + 23.0, '=');

    world_line(c, 125.0, gy, 178.0, gy + 27.0, '/');
    world_line(c, 178.0, gy + 27.0, 231.0, gy, '\\');
    world_line(c, 139.0, gy, 217.0, gy, '=');
    world_text(c, 151.0, gy + 12.0, "LOX");

    world_line(c, -245.0, gy, -330.0, gy - 12.0, '_');
    world_line(c, -330.0, gy - 12.0, -430.0, gy - 7.0, '_');
    world_line(c, 240.0, gy, 390.0, gy - 10.0, '_');

    if (s->t < 0.0) {
        int ph = (int)(-s->t * 7.0) & 3;
        world_text(c, -180.0, gy + 17.0, ph & 1 ? " ~~  ~~" : "  ~~  ~~");
        world_text(c,  42.0, gy + 14.0, ph & 2 ? ".~~ ." : " . ~~");
        world_text(c, -18.0, gy + 28.0, ph & 1 ? "VENT" : "    ");
    }

    if (s->t < 1.0 && s->throttle < 0.05) {
        world_text(c, -20.0, gy + 9.0, "T-READY");
    }
}

static void draw_trail(const Camera *c, const Sim *s)
{
    unsigned i;
    unsigned limit = s->y > 24000.0 ? 260U : s->y > 9000.0 ? 520U : 1400U;

    for (i = 0; i < s->trail_n && i < TRAIL && i < limit; i += (s->y > 9000.0 ? 4U : 2U)) {
        unsigned age = s->trail_n - 1U - i;
        unsigned k = age & (TRAIL - 1U);
        char ch = i < 60 ? ':' : i < 180 ? '.' : '`';
        int x, y;
        map_world(c, s->trail_x[k], s->trail_y[k], &x, &y);
        if ((unsigned)x < (unsigned)fb.w && (unsigned)y < (unsigned)fb.h && at(x, y) == ' ') {
            pix(x, y, ch);
        }
    }
}

static void draw_hud(const Sim *s)
{
    double speed = hypot(s->vx, s->vy);
    double mach = speed / CLAMP(340.0 - 0.0030 * s->y, 295.0, 340.0);
    double q = 0.5 * 1.225 * exp(-fmax(0.0, s->y) / 8400.0) * speed * speed;
    int bar_w = fb.w >= 100 ? 22 : 16;
    int bar_x = 22;
    int bar = (int)CLAMP(s->throttle * (double)bar_w + 0.5, 0.0, (double)bar_w);
    int i;

    hline(0, fb.w - 1, 0, ' ');
    hline(0, fb.w - 1, 1, ' ');
    hline(0, fb.w - 1, 2, ' ');
    if (s->t < 0.0) {
        fmt(0, 0, "T-%7.1fs  ALT %9.0fm  VY %+8.1f  VX %+8.1f  M%4.2f  Q %6.0fPa",
            -s->t, s->y, s->vy, s->vx, mach, q);
    } else {
        fmt(0, 0, "T+%7.1fs  ALT %9.0fm  VY %+8.1f  VX %+8.1f  M%4.2f  Q %6.0fPa",
            s->t, s->y, s->vy, s->vx, mach, q);
    }
    fmt(0, 1, "FUEL %8.0fkg  THR [", s->fuel);
    for (i = 0; i < bar_w; ++i) {
        pix(bar_x + i, 1, i < bar ? '#' : '.');
    }
    fmt(bar_x + bar_w, 1, "] %3.0f%%  Qmax %.0fkPa  Gmax %.2f",
        s->throttle * 100.0, s->max_q / 1000.0, s->max_g);
    if (fb.w >= 104) {
        fmt(82, 1, "wind %+4.0fm/s", s->wind0);
    }
    fmt(0, 2, "w/s throttle  x cutoff  r max  p pause  q abort  target %.0fm",
        s->target);

    if (s->paused) {
        text(fb.w / 2 - 4, fb.h / 2, "PAUSED");
    }
}

static void draw_overlay(const char *msg, const char *sub)
{
    int x = fb.w / 2 - (int)strlen(msg) / 2;
    int y = fb.h / 2 - 1;
    hline(x - 4, x + (int)strlen(msg) + 3, y - 1, '=');
    text(x, y, msg);
    if (sub) {
        text(fb.w / 2 - (int)strlen(sub) / 2, y + 1, sub);
    }
    hline(x - 4, x + (int)strlen(msg) + 3, y + 2, '=');
}

static void render(const Sim *s)
{
    Camera c = camera_for(s);
    int rx, ry;

    draw_sky(&c, s);
    draw_distant_ridges(&c);
    draw_trail(&c, s);
    draw_terrain(&c);
    draw_pad(&c, s);

    map_world(&c, s->x, s->y, &rx, &ry);
    if (c.sy < 35.0) {
        clear_box(rx - 8, ry - 14, rx + 8, ry + 11);
    } else if (c.sy < 180.0) {
        clear_box(rx - 4, ry - 8, rx + 4, ry + 7);
    } else if (c.sy < 900.0) {
        clear_box(rx - 2, ry - 4, rx + 2, ry + 3);
    } else {
        clear_box(rx - 1, ry - 3, rx + 1, ry + 2);
    }
    draw_rocket(rx, ry, s->fuel > 0.0 ? s->throttle : 0.0, c.sy, s->seed + (unsigned)(s->t * 60.0));
    draw_hud(s);

    if (s->success) {
        draw_overlay("TARGET ALTITUDE ACHIEVED", "vehicle is still climbing");
    } else if (s->crash) {
        draw_overlay("RUD", "rapid unscheduled disassembly");
    } else if (s->landed) {
        draw_overlay("SOFT TOUCHDOWN", "unexpectedly civilized");
    }
}

static double ask(const char *name, double def, double lo, double hi)
{
    char buf[128];
    double v;

    printf("%-25s [%g] : ", name, def);
    fflush(stdout);
    if (fgets(buf, sizeof buf, stdin) && sscanf(buf, "%lf", &v) == 1) {
        return CLAMP(v, lo, hi);
    }
    return def;
}

static void push_trail(Sim *s)
{
    s->trail_x[s->trail_n & (TRAIL - 1U)] = s->x;
    s->trail_y[s->trail_n & (TRAIL - 1U)] = s->y;
    ++s->trail_n;
}

static void step(Sim *s, double dt)
{
    double alt = fmax(0.0, s->y);
    double wind = s->wind0 * exp(-alt / 15000.0) +
                  14.0 * sin(s->t * 0.071 + alt * 0.00031) * exp(-alt / 38000.0);
    double rho = 1.225 * exp(-alt / 8400.0);
    double mass = fmax(1.0, s->dry + s->fuel);
    double throttle = s->fuel > 0.0 ? s->throttle : 0.0;
    double isp = 282.0 + 46.0 * (1.0 - rho / 1.225);
    double thrust = 880000.0 * throttle;
    double flow = throttle > 0.0 ? thrust / (isp * G0) : 0.0;
    double burn = flow * dt;
    double rvx, rvy, speed_rel, speed, q, drag, cd_area, g, ax, ay, gload;

    if (burn > s->fuel) {
        burn = s->fuel;
    }
    s->fuel -= burn;

    rvx = s->vx - wind;
    rvy = s->vy;
    speed_rel = hypot(rvx, rvy) + 1e-9;
    speed = hypot(s->vx, s->vy);
    q = 0.5 * rho * speed_rel * speed_rel;
    cd_area = 8.5;
    drag = q * cd_area;
    g = G0 * (EARTH_R / (EARTH_R + alt)) * (EARTH_R / (EARTH_R + alt));

    ax = -drag * rvx / speed_rel / mass;
    ay = thrust / mass - drag * rvy / speed_rel / mass - g;
    gload = hypot(ax, ay + g) / G0;

    s->vx += ax * dt;
    s->vy += ay * dt;
    s->x += s->vx * dt;
    s->y += s->vy * dt;
    s->t += dt;

    push_trail(s);

    if (q > s->max_q) {
        s->max_q = q;
    }
    if (gload > s->max_g) {
        s->max_g = gload;
    }
    if (s->y > s->max_alt) {
        s->max_alt = s->y;
    }
    if (speed > s->max_speed) {
        s->max_speed = speed;
    }

    if (s->y <= terrain(s->x) && s->t > 0.8) {
        s->y = terrain(s->x);
        s->done = 1;
        if (fabs(s->vy) < 7.5 && fabs(s->vx) < 6.0) {
            s->landed = 1;
        } else {
            s->crash = 1;
        }
    }
    if (s->y >= s->target && s->vy > 0.0) {
        s->done = 1;
        s->success = 1;
    }
    if (s->t > 1800.0) {
        s->done = 1;
    }
}

static void keys(Sim *s)
{
    int c;
    while ((c = get_key()) != 0) {
        if (c == 'q' || c == 27 || c == 3) {
            s->done = 1;
            s->aborted = 1;
        } else if (c == 'w' || c == 'W') {
            s->throttle = CLAMP(s->throttle + 0.035, 0.0, 1.0);
        } else if (c == 's' || c == 'S') {
            s->throttle = CLAMP(s->throttle - 0.035, 0.0, 1.0);
        } else if (c == 'r' || c == 'R') {
            s->throttle = 1.0;
        } else if (c == 'x' || c == 'X') {
            s->throttle = 0.0;
        } else if (c == 'p' || c == 'P' || c == ' ') {
            s->paused = !s->paused;
        }
    }
}

static int countdown_keys(Sim *s)
{
    int c;

    while ((c = get_key()) != 0) {
        if (c == 'q' || c == 27 || c == 3) {
            s->done = 1;
            s->aborted = 1;
            return 0;
        }
    }
    return 1;
}

static void big_digit(int d, int cx, int y)
{
    static const char *g[10][5] = {
        {" ### ", "#   #", "#   #", "#   #", " ### "},
        {"  #  ", " ##  ", "  #  ", "  #  ", " ### "},
        {" ### ", "#   #", "   # ", "  #  ", "#####"},
        {"#### ", "    #", " ### ", "    #", "#### "},
        {"#   #", "#   #", "#####", "    #", "    #"},
        {"#####", "#    ", "#### ", "    #", "#### "},
        {" ### ", "#    ", "#### ", "#   #", " ### "},
        {"#####", "    #", "   # ", "  #  ", "  #  "},
        {" ### ", "#   #", " ### ", "#   #", " ### "},
        {" ### ", "#   #", " ####", "    #", " ### "}
    };
    int r;

    d = CLAMP(d, 0, 9);
    for (r = 0; r < 5; ++r) {
        text_centered(cx, y + r, g[d][r]);
    }
}

static void draw_countdown_overlay(double rem)
{
    int d = (int)ceil(rem);
    int cx = CLAMP(fb.w / 4, 14, fb.w - 14);
    int y = CLAMP(fb.h / 2 - 7, 5, fb.h - 9);

    if (rem > 1.0) {
        text_centered(cx, y - 2, "AUTO SEQUENCE START");
        big_digit(d, cx, y);
        text_centered(cx, y + 6, "T-MINUS");
    } else {
        text_centered(cx, y - 1, "IGNITION");
        text_centered(cx, y + 1, "HOLD-DOWNS RELEASE");
    }
}

static int run_countdown(Sim *s)
{
    const double total = 5.0;
    double start = now_s();
    int w = 0, h = 0;

    while (!s->done) {
        double elapsed = now_s() - start;
        double rem = total - elapsed;
        Sim v = *s;

        if (rem <= 0.0) {
            break;
        }

        term_size(&w, &h);
        if (!frame_make(w, h)) {
            s->done = 1;
            s->aborted = 1;
            return 0;
        }

        if (!countdown_keys(s)) {
            break;
        }

        v.t = -rem;
        v.throttle = rem < 1.0 ? s->throttle * (1.0 - rem) : 0.0;
        render(&v);
        draw_countdown_overlay(rem);
        frame_diff();
        sleep_ms(24);
    }

    s->t = 0.0;
    return 1;
}

static void init_sim(Sim *s)
{
    memset(s, 0, sizeof *s);
    s->fuel = ask("propellant kg", 30000.0, 1000.0, 180000.0);
    s->dry = ask("dry mass + payload kg", 8500.0, 750.0, 90000.0);
    s->wind0 = ask("surface wind m/s", 0.0, -90.0, 90.0);
    s->throttle = ask("initial throttle pct", 92.0, 0.0, 100.0) / 100.0;
    s->target = ask("target altitude m", 80000.0, 1000.0, 500000.0);
    s->seed = mix32((uint32_t)(s->fuel * 17.0) ^ (uint32_t)(s->dry * 31.0) ^
                    (uint32_t)((s->wind0 + 100.0) * 1000.0));
    s->y = terrain(0.0) + 23.0;
    push_trail(s);
}

static void print_result(const Sim *s)
{
    if (s->success) {
        puts("Target altitude achieved.");
    } else if (s->crash) {
        puts("RUD: rapid unscheduled disassembly.");
    } else if (s->landed) {
        puts("Soft touchdown.");
    } else if (s->aborted) {
        puts("Aborted.");
    } else {
        puts("Simulation ended.");
    }
    printf("final: T+%.1fs alt %.0fm vx %.1f vy %.1f fuel %.0fkg drift %.0fm maxQ %.0fPa maxG %.2f maxV %.1fm/s\n",
           s->t, s->y, s->vx, s->vy, s->fuel, s->x, s->max_q, s->max_g, s->max_speed);
}

static void run_headless(Sim *s)
{
    while (!s->done) {
        step(s, 1.0 / 120.0);
    }
}

int main(void)
{
    Sim sim;
    int w = 0, h = 0;
    double last, acc = 0.0;

    puts("rocket.c: low-level ASCII vector launch simulator");
    init_sim(&sim);

    if (!isatty(STDOUT_FILENO)) {
        run_headless(&sim);
        print_result(&sim);
        return sim.crash ? 2 : 0;
    }

    signal(SIGINT, die_clean);
    signal(SIGTERM, die_clean);
    atexit(term_end);
    term_begin();

    if (!run_countdown(&sim)) {
        term_end();
        frame_free();
        print_result(&sim);
        return 1;
    }

    last = now_s();
    while (!sim.done) {
        double n = now_s();
        double dt = n - last;
        last = n;
        if (dt > 0.08) {
            dt = 0.08;
        }
        acc += dt;

        term_size(&w, &h);
        if (!frame_make(w, h)) {
            term_end();
            fprintf(stderr, "Need at least %dx%d terminal cells.\n", MIN_W, MIN_H);
            frame_free();
            return 1;
        }

        keys(&sim);
        while (acc >= 1.0 / 60.0) {
            if (!sim.paused) {
                step(&sim, 1.0 / 60.0);
            }
            acc -= 1.0 / 60.0;
        }

        render(&sim);
        frame_diff();
        sleep_ms(5);
    }

    render(&sim);
    frame_diff();
    sleep_ms(450);
    term_end();
    frame_free();
    print_result(&sim);
    return sim.crash ? 2 : 0;
}
