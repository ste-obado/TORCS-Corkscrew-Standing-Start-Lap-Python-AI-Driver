# Submission Notes — snakeoil3_v3_2.py

## What this driver does

`snakeoil3_v3_2.py` is a hand-tuned rule-based SCR client for TORCS.  
It uses no machine learning, no neural network, and no external libraries.  
All logic is computed from the 19 track-range sensors and the standard SCR
state packet each step.

---

## Driver configuration (exact values as submitted)

| Constant | Value | Note |
|---|---|---|
| `_DECEL` | 14.0 m/s² | braking trust; ABS active |
| `_CORNER_MARGIN` | — (absent) | no safety margin subtracted |
| `_MAX_SPEED` | 250.0 km/h | hard cap |
| `_MIN_SPEED` | 50.0 km/h | speed floor |
| `_GRIP` | — (absent) | no grip multiplier |
| `_ACCEL_GAIN` | 0.08 | throttle proportional gain |
| `_BRAKE_GAIN` | 0.12 | brake proportional gain |
| `_BRAKE_DEADBAND` | 3.0 km/h | braking tolerance |
| `_STEER_DIVISOR` | 0.5 | steering gain (2.0 rad⁻¹) |
| `_STEER_TRACK_BASE` | 0.30 | centre-pull at high speed |
| `_STEER_TRACK_EXTRA` | 0.50 | extra centre-pull at low speed |
| `_TCS_BASE_THRESHOLD` | 3.0 rad/s | traction control base threshold |
| `_DOWNSHIFT_RPM` | 5500 | downshift trigger |
| `_UPSHIFT_RPM` | 8500 | upshift trigger |
| `_WHEEL_RADIUS` | 0.3179 m | ABS wheel radius |

---

## Techniques used

| Technique | Present | Details |
|---|---|---|
| Physics-based braking curve | yes | `target = 3.6 * sqrt(2 * _DECEL * forward)` |
| ABS (anti-lock braking) | yes | halves brake when wheel speed < car speed by >2 m/s |
| Speed-variable centre spring | yes | k_pos ramps 0.30→0.80 as speed drops 120→60 km/h |
| RPM-based gear changes | yes | up at 8500, down at 5500 |
| Traction control | yes | cuts throttle when rear spin exceeds speed-scaled threshold |
| Corner anticipation bias | yes | wide-angle beams (indices 0,1,17,18) used for pre-steering |
| GRIP multiplier | no | not present in v3.2 |
| Corner safety margin | no | not present in v3.2 |

---

## Experiment results

### Version history and measured lap performance

| Version | Key change | Observed top speed | Corner speeds (approx) | Lap time | Notes |
|---|---|---|---|---|---|
| v3 (baseline) | ABS, RPM gears, throttled telemetry | not recorded | not recorded | not recorded | |
| v3.1 | Speed-variable centre spring (k_pos), DOWNSHIFT 5500 | not recorded | not recorded | not recorded | |
| **v3.2 (submitted)** | STEER_TRACK_EXTRA 0.40→0.50, DECEL 13→14 | **213 km/h** | 51–86 km/h (9 corners) | not recorded | top speed noted in session |
| v3.3 | GRIP 1.10, DECEL→15, STEER_DIVISOR→0.4 | not recorded | not recorded | not recorded | zig-zag on straights; not submitted |
| v3.4 | Reverted divisor to 0.5, CORNER_MARGIN 15, TCS 3.0 restored | not recorded | not recorded | not recorded | wheelspin fix at dist ~1943 m |

### Corner speed profile (v3.2 — 9 corners measured)

| Corner | Measured speed (km/h) |
|---|---|
| 1 | 59 |
| 2 | 69 |
| 3 | 75 |
| 4 | 82 |
| 5 | 70 |
| 6 | 52 |
| 7 | 86 |
| 8 | 76 |
| 9 | 51 |

Root cause identified (corners 6 and 9 at ~51–52 km/h):  
The forward beams (±1.7°) saw only 8–24 m ahead in corners because `trackPos`
drifted to |0.62–0.86|, pushing the car into the outer wall view.
The centre-spring stiffening in v3.1/v3.2 was the primary fix.

---

## Known issues

| Issue | Status |
|---|---|
| Wheelspin at dist ~1943 m (RPM 14k–19k at 45–51 km/h) | Identified in later session; not present in v3.2 (TCS threshold was 3.0) |
| Corner 6 and 9 at ~51 km/h (near floor) | Partially addressed in v3.2 via stiffer spring; root cause was outer-wall drift |
| Zig-zagging on straights with STEER_DIVISOR 0.4 | Found in v3.3 testing; v3.2 uses 0.5 — not present here |

---

## Evidence

| File | Status |
|---|---|
| `evidence/run.log` | not recorded — no log file was captured and saved |
| `evidence/*.png` | not recorded — no screenshots were saved |

To capture evidence on the next run:

```bash
python snakeoil3_v3_2.py 2>&1 | tee evidence/run.log
```

---

## Setup requirements not included in this folder

| Component | Where to get it |
|---|---|
| TORCS simulator | system package or build from source (Linux) |
| SCR server plugin | included with vtorcs-RL-color or standard TORCS builds |
| vtorcs-RL-color | compile from `gym_torcs/vtorcs-RL-color/` in parent repo |
| xautomation (`xte`) | `sudo apt-get install xautomation` on Ubuntu |

This driver does **not** require OpenAI Gym, numpy, TensorFlow, or any other
third-party Python package.
