# evidence/

This folder is reserved for run logs and screenshots captured during testing.

## Expected contents

| Filename | How to capture |
|---|---|
| `run.log` | `python snakeoil3_v3_2.py 2>&1 \| tee evidence/run.log` |
| `screenshot_*.png` | Manual screenshot during TORCS race window |

## Current status


All telemetry was observed live in the terminal and is not recorded here.

Known data points recorded from observation (see SUBMISSION.md for detail):
- Top speed: 213 km/h (v3.2 session)
- Corner speeds (9 corners): 59, 69, 75, 82, 70, 52, 86, 76, 51 km/h
- Wheelspin observed at dist ~1943 m in v3.3 (RPM 14,000–19,700 at 45–51 km/h)
