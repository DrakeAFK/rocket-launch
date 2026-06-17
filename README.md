# rocket.c

terminal rocket launch simulator written in C

## Build

```sh
gcc -O2 -std=c99 -Wall -Wextra rocket.c -lm -o rocket
```

Requires a POSIX-ish system and a terminal that understands ANSI escape sequences.

## Run

```sh
./rocket
```

At startup, the program asks for:

```text
propellant kg
dry mass + payload kg
surface wind m/s
initial throttle pct
target altitude m
```

Press Enter to accept a default.

## Controls

| Key | Action |
| --- | ------ |
| `w` | throttle up |
| `s` | throttle down |
| `x` | engine cutoff |
| `r` | max throttle |
| `p` / space | pause |
| `q` / Esc / Ctrl-C | abort |

## Headless mode

Pipe the inputs to run without the interactive renderer:

```sh
printf '30000\n8500\n0\n92\n80000\n' | ./rocket
```

Example output:

```text
Target altitude achieved.
final: T+106.8s alt 80004m vx 0.3 vy 2411.6 fuel 1733kg drift 81m maxQ 34404Pa maxG 8.04 maxV 2411.0m/s
```

## What it does

- Fixed-step flight integration.
- Exponential atmosphere model.
- Gravity decreases with altitude.
- Thrust, fuel flow, drag, wind, dynamic pressure, and g-load.
- Terrain collision with soft-touchdown detection.
- Ring-buffered flight trail.
- Procedural terrain, cloud decks, stars, and horizon/earth-limb rendering.
- Terminal double-buffering with diff redraws instead of full-frame spam.


## Exit codes

| Code | Meaning |
| ---- | ------- |
| `0` | simulation ended without crash |
| `1` | setup/countdown/terminal failure |
| `2` | crash / RUD |
