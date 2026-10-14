# AI Racing League final project
 
## Author and members
 
| Name | Contribution |
|------|--------------|
| Stephen Odero Ochieng | Individual project. Wrote and tuned the driver, ran and timed every lap, read the telemetry logs, decided which changes to keep or reject, recorded the video, built this repository. |
 
## Problem statement
 
The task is to drive a car around the Corkscrew track in TORCS from a standing start as fast as possible, using a Python SCR client that only sees the track-range sensors and state values the server sends every step. My first driver, built from the kit's `snakeoil3_gym.py` demo logic, crawled at about 39 km/h on straights and reached a top speed of only 70 km/h. I rewrote the driving logic with a stopping-distance speed target, active braking with ABS, a corrected steering gain, RPM-based gears, a speed-dependent pull to the road centre and traction control, and tuned it over many timed laps from 2:29 to a repeatable 1:49 to 1:52.
 
## Environment and setup
 
- OS: Windows [CHECK: Windows 10 or 11]
- Python: 3.14 (64-bit), standard library only
- TORCS and challenge kit: `gym_torcs` kit folder (naoto yoshida, MIT) with TORCS patched with the SCR server plugin (`scr_server` driver, UDP port 3001). [CHECK: TORCS version and where it was installed from]
  - Race configuration: Race > Quick Race > Configure Race. Track: Corkscrew. Drivers: only `scr_server 1` selected (all other `scr_server` entries removed). Standing start. Default livery. No car model or physics files edited.
- Required local files: `snakeoil3_v3_2.py` only. It is self-contained and has no config files or assets. `autostart.sh` from the kit is Linux-only and is not used.
- External dependencies: none. Python 3 and TORCS with the SCR plugin are enough, and there is nothing to install with pip.
## Run
 
From the folder that contains `snakeoil3_v3_3.py` (for example `gym_torcs`), start TORCS by hand:
 
1. Open TORCS and go to Race > Quick Race > Configure Race. Choose the Corkscrew track and select only `scr_server 1` as the driver, then start the race.
2. Wait until the TORCS console shows `Waiting for request on port 3001`. The TORCS window looks "Not Responding" while it waits, which is normal.
3. In a separate terminal, start the driver.
Windows command: `py snakeoil3_v3_2.py` (no launch options needed; defaults are host `localhost`, port `3001`, id `SCR`)
 
macOS/Linux command: `python3 snakeoil3_v3_2.py`
 
The simulator starts first and the client second. The client retries the handshake every second, so it connects as soon as the race has finished loading. The kit's Linux-only relaunch code (`pkill`, `torcs`, `sh autostart.sh`) only prints harmless errors on Windows. 
 
## Results
 
Settings for every run: Corkscrew track, standing start, single car (`scr_server 1`), default livery, driver started after the race was waiting on port 3001.
 
| Run | Lap time | Top speed | Notes |
|-----|----------|-----------|-------|
| Baseline kit driver (`snakeoil3_gym.py` demo logic) | not timed | not recorded | about 39 km/h on straights in my first version |
| Submitted driver `snakeoil3_v3_2.py`, run 1 | 1:49 [CHECK exact time] | [CHECK] | clean, no crash |
| Submitted driver `snakeoil3_v3_2.py`, run 2 | [CHECK exact time] | [CHECK] | clean, no crash |
| Submitted driver `snakeoil3_v3_2.py`, run 3 | 1:52 [CHECK exact time] | 213 km/h | clean, no crash |
 
Three repeat runs of v5 stayed between 1:49 and 1:52. Damage: [CHECK: damage value or "not recorded"].
 
I chose v5 over a faster version (v6) because v6 was not repeatable: three runs gave 1:43, 2:00 and 2:09.
 
Corner speeds measured on the v3 lap (9 corners): 59, 69, 75, 82, 70, 52, 86, 76 and 51 km/h.
 
See `experiment_results.csv` for the full version history and `evidence/` for [CHECK: add the actual evidence paths, for example `evidence/lap_log_v5.txt` and the video file or link].
 
## Enhancements
 
1. Stopping-distance speed target. The target speed is `3.6 * sqrt(2 * _DECEL * forward)`, where `forward` is the longest of the seven centre track beams. This replaced a linear rule that read the near-side beams and held the car near 70 km/h on straights. Evidence: with the steering fix below, top speed rose from 70 to 172 km/h and the lap went from not completing a fast lap to 2:29 with no crash.
2. Corrected steering gain. My v1 steering was about 25 times weaker than the kit's, so the car could not hold corners at speed. v2 onward uses `steer = angle / 0.5`. Evidence: 2:29 and 2:06 laps with no crash at 172 to 201 km/h.
3. ABS and braking. Brake pressure is halved when the mean wheel surface speed is more than 2 m/s below the car speed (lock-up). This allowed a higher `_DECEL` (13). Evidence: the lap fell to 2:00.058 with a top speed of 250 km/h. This change was made together with the speed cap raise, so the gain is not isolated.
4. Speed-dependent pull to the road centre. In the v3 log the car drifted to `trackPos` of 0.6 to 0.86 in corners, the outer wall shortened the forward beams to 8 to 24 m, and the target dropped to 51 to 53 km/h. A stiffer pull at low speed fixed this. Evidence: the lap improved from 2:02 to 1:52 and the repeat runs stayed at 1:49 to 1:52.
5. RPM-based gears and speed-scaled traction control. Gears shift on RPM instead of a hard-coded gear 1, and traction control cuts throttle when the rear wheels spin faster than the fronts by a speed-scaled threshold. Evidence: logs show the engine staying in a normal rev range through corner exits.
Changes I tried and rejected: wider look-ahead beams, a halved centre pull and a higher speed floor (v4, 2:10 with wide corners); and a variant with extra braking margin, a lower floor and stronger low-speed steering (zig-zag and a crash at about 2050 m).
 
## Use of IBM Bob

IBM Bob: generated all driver versions from `snakeoil3_v1.py` through `snakeoil3_v3_4.py`, implementing each change requested in the session. Bob wrote the `_abs_brake()`, `_corner_speed_target()`, `_corner_geometry()`, and `_rpm_gear()` helper functions, the telemetry print logic, and the submission folder structure including `README.md`, `SUBMISSION.md`, and `experiment_results.csv`. Bob also diagnosed root causes from the telemetry I pasted (outer-wall drift, wheelspin at dist ~1943 m, floor-limited corner speeds) and proposed the fixes I then tested on-track.
 
Claude (Anthropic chat assistant): proposed the code changes described above, explained the TORCS connection problem, and helped read my telemetry logs. It also gave some wrong suggestions that I tested and rejected, for example raising the speed floor and widening the beams (v4).
 
What I did myself: ran and timed every lap, pasted the telemetry, decided which changes to keep, reverted the ones that made the car slower or unstable, ran repeat laps to compare v5 and v6, and chose v5 as the submission because it was consistent.
 
## Demonstration
 
[Watch the submitted driver on YouTub](https://youtu.be/NqQpUMUOD6A)
 
## Limitations
 
- Lap time varies by about 3 seconds between runs (1:49 to 1:52 over three runs), and I tested only three runs of v5.
- Tuned and tested on Corkscrew only. Other tracks were not tested.
- Sharp corners still run slowly (about 51 to 55 km/h at the floor), and in some experimental versions the car ran off the road at about 2490 m and 2050 m. v5 may still run wide in these corners.
- v6 reached 1:43 but was inconsistent (2:00 and 2:09 on other runs), so it is not the submitted version.
- Damage per lap and a baseline lap time for the kit driver were not recorded.
- Tested on Windows only.
- Next tests: five repeat runs of any new version before keeping it; a braking margin for corners that appear late; checking the corner sign of the look-ahead bias; recording a baseline lap for comparison.
## Credits and licensing
 
| Component | Author / source | Licence |
|-----------|-----------------|---------|
| `gym_torcs` kit, `snakeoil3_gym.py` structure | naoto yoshida (gym-torcs) | MIT |
| `Client`, `ServerState`, `DriverAction`, `destringify`, `bargraph`, `clip` | Chris X Edwards, snakeoil python client for TORCS | see source header |
| TORCS simulator and SCR server plugin | Bernhard Wymann et al. and the SCR competition organisers | GPL v2 |
| `snakeoil3_v5.py` driving logic and tuning constants | Stephen Odero Ochieng, written with help from AI assistants (see above) | MIT, same terms as the kit |
 
The networking and data classes are carried forward from the provided kit with small Python 3 fixes. I do not claim ownership of the provided kit code, only of the driving logic and tuning.
 
AI Racing League | Sessions 1-4 | Learner handbook 20
 