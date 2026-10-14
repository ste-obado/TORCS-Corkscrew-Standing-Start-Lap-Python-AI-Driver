#!/usr/bin/python
# snakeoil3_v3_2.py
# Based on snakeoil3_v3_1.py
#
# v3.2 — one-change-per-lap tuning pass  (top speed 213, gains come from corners)
#
# What v3.1 established
# ---------------------
# spd tracks tgt cleanly — braking and throttle are not the bottleneck.
# fwd 8-24 m, pos drifted to |0.62-0.86| — outer-wall drift is still the root cause.
# _DOWNSHIFT_RPM = 5500 confirmed or pending confirmation from CORNER print.
# Top speed 213 km/h — well below the 250 cap, so _MAX_SPEED is not limiting.
# All remaining gains come from keeping the car centred through corners and
# braking later into them.
#
# v3.2 change 1 — Stiffer centre spring: _STEER_TRACK_EXTRA 0.40 → 0.50
#   k_pos at <=60 km/h: 0.30 + 0.50 = 0.80  (was 0.70 in v3.1)
#   k_pos at >=120 km/h: 0.30               (unchanged)
#   Target: |pos| at corner exit < 0.5.  If the car weaves, revert to 0.40.
#
# v3.2 change 2 — Later braking: _DECEL 13.0 → 14.0 m/s^2
#   With ABS preventing lock-up, the car can trust higher deceleration.
#   Effect: braking point moves ~5-8 m later per corner.
#   Revert to 13.0 if the car runs wide on corner entry.
#   Try 15.0 next lap if 14.0 is clean.

import socket
import sys
import getopt
import os
import time
import math

PI = math.pi

data_size = 2**17

# ---------------------------------------------------------------------------
# CLI help strings
# ---------------------------------------------------------------------------
ophelp  = 'Options:\n'
ophelp += ' --host, -H <host>    TORCS server host. [localhost]\n'
ophelp += ' --port, -p <port>    TORCS port. [3001]\n'
ophelp += ' --id, -i <id>        ID for server. [SCR]\n'
ophelp += ' --steps, -m <#>      Maximum simulation steps. 1 sec ~ 50 steps. [100000]\n'
ophelp += ' --episodes, -e <#>   Maximum learning episodes. [1]\n'
ophelp += ' --track, -t <track>  Your name for this track. Used for learning. [unknown]\n'
ophelp += ' --stage, -s <#>      0=warm up, 1=qualifying, 2=race, 3=unknown. [3]\n'
ophelp += ' --debug, -d          Output full telemetry.\n'
ophelp += ' --help, -h           Show this help.\n'
ophelp += ' --version, -v        Show current version.'
usage   = 'Usage: %s [ophelp [optargs]] \n' % sys.argv[0]
usage   = usage + ophelp
version = "v3.2-2024"


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def clip(v, lo, hi):
    """Clamp v to [lo, hi]."""
    if v < lo:   return lo
    elif v > hi: return hi
    else:        return v


def bargraph(x, mn, mx, w, c='X'):
    '''Draws a simple ascii-art bar graph.
    x=value, mn=min, mx=max, w=width in chars, c=fill character.'''
    if not w: return ''
    if x < mn: x = mn
    if x > mx: x = mx
    tx = mx - mn
    if tx <= 0: return 'backwards'
    upw = tx / float(w)
    if upw <= 0: return 'what?'
    negpu, pospu, negnonpu, posnonpu = 0, 0, 0, 0
    if mn < 0:
        if x < 0:
            negpu    = -x + min(0, mx)
            negnonpu = -mn + x
        else:
            negnonpu = -mn + min(0, mx)
    if mx > 0:
        if x > 0:
            pospu    = x - max(0, mn)
            posnonpu = mx - x
        else:
            posnonpu = mx - max(0, mn)
    nnc = int(negnonpu / upw) * '-'
    npc = int(negpu    / upw) * c
    ppc = int(pospu    / upw) * c
    pnc = int(posnonpu / upw) * '_'
    return '[%s]' % (nnc + npc + ppc + pnc)


# ---------------------------------------------------------------------------
# Client
# ---------------------------------------------------------------------------

class Client():
    def __init__(self, H=None, p=None, i=None, e=None, t=None, s=None,
                 d=None, vision=False):
        self.vision = vision

        self.host        = 'localhost'
        self.port        = 3001
        self.sid         = 'SCR'
        self.maxEpisodes = 1
        self.trackname   = 'unknown'
        self.stage       = 3      # 0=Warm-up, 1=Qualifying, 2=Race, 3=unknown
        self.debug       = False
        self.maxSteps    = 100000  # 50 steps/second

        # Only parse argv when running as a standalone script, not when
        # imported by a Gym environment (which passes its own argv).
        if __name__ == '__main__':
            self.parse_the_command_line()

        if H: self.host        = H
        if p: self.port        = p
        if i: self.sid         = i
        if e: self.maxEpisodes = e
        if t: self.trackname   = t
        if s: self.stage       = s
        if d: self.debug       = d

        self.S = ServerState()
        self.R = DriverAction()
        self.setup_connection()

    # ------------------------------------------------------------------
    def setup_connection(self):
        '''Open UDP socket and perform the SCR identification handshake.'''
        try:
            self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error:
            print('Error: Could not create socket...')
            sys.exit(-1)

        self.so.settimeout(1)
        n_fail = 5

        while True:
            # 19 non-uniform track sensor angles (degrees), denser near centre.
            # Index mapping (left to right):
            #   0=-45  1=-19  2=-12  3=-7  4=-4  5=-2.5  6=-1.7  7=-1  8=-.5
            #   9=0  10=.5  11=1  12=1.7  13=2.5  14=4  15=7  16=12  17=19  18=45
            a = "-45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45"
            initmsg = '%s(init %s)' % (self.sid, a)

            try:
                self.so.sendto(initmsg.encode(), (self.host, self.port))
            except socket.error:
                sys.exit(-1)

            sockdata: str = ''
            try:
                raw, addr = self.so.recvfrom(data_size)
                sockdata = raw.decode('utf-8')
            except socket.error:
                print("Waiting for server on %d............" % self.port)
                print("Count Down : " + str(n_fail))
                if n_fail < 0:
                    print("relaunch torcs")
                    os.system('pkill torcs')
                    time.sleep(1.0)
                    if self.vision is False:
                        os.system('torcs -nofuel -nodamage -nolaptime &')
                    else:
                        os.system('torcs -nofuel -nodamage -nolaptime -vision &')
                    time.sleep(1.0)
                    os.system('sh autostart.sh')
                    n_fail = 5
                n_fail -= 1

            if '***identified***' in sockdata:
                print("Client connected on %d.............." % self.port)
                break

    # ------------------------------------------------------------------
    def parse_the_command_line(self):
        try:
            (opts, args) = getopt.getopt(
                sys.argv[1:], 'H:p:i:m:e:t:s:dhv',
                ['host=', 'port=', 'id=', 'steps=',
                 'episodes=', 'track=', 'stage=',
                 'debug', 'help', 'version'])
        except getopt.error as why:
            print('getopt error: %s\n%s' % (why, usage))
            sys.exit(-1)
        opt = ('', '')
        try:
            for opt in opts:
                if opt[0] in ('-h', '--help'):
                    print(usage); sys.exit(0)
                if opt[0] in ('-d', '--debug'):
                    self.debug = True
                if opt[0] in ('-H', '--host'):
                    self.host = opt[1]
                if opt[0] in ('-i', '--id'):
                    self.sid = opt[1]
                if opt[0] in ('-t', '--track'):
                    self.trackname = opt[1]
                if opt[0] in ('-s', '--stage'):
                    self.stage = int(opt[1])
                ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
                if opt[0] in ('-p', '--port'):
                    self.port = int(opt[1])
                ######## END OF REQUIRED #########
                if opt[0] in ('-e', '--episodes'):
                    self.maxEpisodes = int(opt[1])
                if opt[0] in ('-m', '--steps'):
                    self.maxSteps = int(opt[1])
                if opt[0] in ('-v', '--version'):
                    print('%s %s' % (sys.argv[0], version)); sys.exit(0)
        except ValueError as why:
            print("Bad parameter '%s' for option %s: %s\n%s" % (
                opt[1], opt[0], why, usage))
            sys.exit(-1)
        if len(args) > 0:
            print('Superfluous input? %s\n%s' % (', '.join(args), usage))
            sys.exit(-1)

    # ------------------------------------------------------------------
    ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
    def get_servers_input(self):
        '''Receive one state packet from the server and parse it.'''
        if not self.so: return
        sockdata: str = ''

        while True:
            try:
                raw, addr = self.so.recvfrom(data_size)
                sockdata = raw.decode('utf-8')
            except socket.error:
                print('.', end=' ')

            if '***identified***' in sockdata:
                print("Client connected on %d.............." % self.port)
                continue
            elif '***shutdown***' in sockdata:
                print(("Server has stopped the race on %d. "
                       "You were in %d place.") %
                      (self.port, self.S.d['racePos']))
                self.shutdown()
                return
            elif '***restart***' in sockdata:
                print("Server has restarted the race on %d." % self.port)
                self.shutdown()
                return
            elif not sockdata:
                continue
            else:
                self.S.parse_server_str(sockdata)
                if self.debug:
                    sys.stderr.write("\x1b[2J\x1b[H")
                    print(self.S)
                break
    ########## END OF REQUIRED #########

    # ------------------------------------------------------------------
    def respond_to_server(self):
        if not self.so: return
        try:
            message = repr(self.R)
            self.so.sendto(message.encode(), (self.host, self.port))
        except socket.error as emsg:
            print("Error sending to server: %s" % str(emsg))
            sys.exit(-1)
        if self.debug: print(self.R.fancyout())

    # ------------------------------------------------------------------
    def shutdown(self):
        if not self.so: return
        print("Race terminated or %d steps elapsed. Shutting down %d."
              % (self.maxSteps, self.port))
        self.so.close()
        self.so = None


# ---------------------------------------------------------------------------
# ServerState
# ---------------------------------------------------------------------------

class ServerState():
    '''What the server is reporting right now.'''
    def __init__(self):
        self.servstr = str()
        self.d       = dict()

    def parse_server_str(self, server_string):
        '''Parse the server string into self.d dictionary.'''
        self.servstr = server_string.strip()[:-1]
        sslisted = self.servstr.strip().lstrip('(').rstrip(')').split(')(')
        for i in sslisted:
            w = i.split(' ')
            self.d[w[0]] = destringify(w[1:])

    def __repr__(self):
        return self.fancyout()

    def fancyout(self):
        '''Specialty output for useful ServerState monitoring.'''
        out = str()
        sensors = [
            'stucktimer',
            'fuel',
            'distRaced',
            'distFromStart',
            'opponents',
            'wheelSpinVel',
            'z',
            'speedZ',
            'speedY',
            'speedX',
            'targetSpeed',
            'rpm',
            'skid',
            'slip',
            'track',
            'trackPos',
            'angle',
        ]
        for k in sensors:
            if type(self.d.get(k)) is list:
                if k == 'track':
                    raw_tsens = ['%.1f' % x for x in self.d['track']]
                    strout = (' '.join(raw_tsens[:9]) + '_' +
                              raw_tsens[9] + '_' +
                              ' '.join(raw_tsens[10:]))
                elif k == 'opponents':
                    strout = str()
                    for osensor in self.d['opponents']:
                        if   osensor > 190: oc = '_'
                        elif osensor >  90: oc = '.'
                        elif osensor >  39: oc = chr(int(osensor / 2) + 97 - 19)
                        elif osensor >  13: oc = chr(int(osensor) + 65 - 13)
                        elif osensor >   3: oc = chr(int(osensor) + 48 - 3)
                        else:               oc = '?'
                        strout += oc
                    strout = ' -> ' + strout[:18] + ' ' + strout[18:] + ' <-'
                else:
                    strout = ', '.join([str(i) for i in self.d[k]])
            else:
                if k == 'gear':
                    gs = '_._._._._._._._._'
                    p  = int(self.d['gear']) * 2 + 2
                    l  = '%d' % self.d['gear']
                    if l == '-1': l = 'R'
                    if l == '0':  l = 'N'
                    strout = gs[:p] + '(%s)' % l + gs[p + 3:]
                elif k == 'damage':
                    strout = '%6.0f %s' % (self.d[k], bargraph(self.d[k], 0, 10000, 50, '~'))
                elif k == 'fuel':
                    strout = '%6.0f %s' % (self.d[k], bargraph(self.d[k], 0, 100, 50, 'f'))
                elif k == 'speedX':
                    cx = 'R' if self.d[k] < 0 else 'X'
                    strout = '%6.1f %s' % (self.d[k], bargraph(self.d[k], -30, 300, 50, cx))
                elif k == 'speedY':
                    strout = '%6.1f %s' % (self.d[k], bargraph(self.d[k] * -1, -25, 25, 50, 'Y'))
                elif k == 'speedZ':
                    strout = '%6.1f %s' % (self.d[k], bargraph(self.d[k], -13, 13, 50, 'Z'))
                elif k == 'z':
                    strout = '%6.3f %s' % (self.d[k], bargraph(self.d[k], .3, .5, 50, 'z'))
                elif k == 'trackPos':
                    cx = '>' if self.d[k] < 0 else '<'
                    strout = '%6.3f %s' % (self.d[k], bargraph(self.d[k] * -1, -1, 1, 50, cx))
                elif k == 'stucktimer':
                    if self.d[k]:
                        strout = '%3d %s' % (self.d[k], bargraph(self.d[k], 0, 300, 50, "'"))
                    else:
                        strout = 'Not stuck!'
                elif k == 'rpm':
                    g = self.d['gear']
                    g = 'R' if g < 0 else '%1d' % g
                    strout = bargraph(self.d[k], 0, 10000, 50, g)
                elif k == 'angle':
                    asyms = [
                        "  !  ", ".|'  ", "./'  ", "_.-  ", ".--  ", "..-  ",
                        "---  ", ".__  ", "-._  ", "'-.  ", r"'\.  ", "'|.  ",
                        "  |  ", "  .|'", "  ./'", "  .-'", "  _.-", "  __.",
                        "  ---", "  --.", "  -._", "  -..", r"  '\.", "  '|."
                    ]
                    rad    = self.d[k]
                    deg    = int(rad * 180 / PI)
                    symno  = int(.5 + (rad + PI) / (PI / 12))
                    symno  = symno % (len(asyms) - 1)
                    strout = '%5.2f %3d (%s)' % (rad, deg, asyms[symno])
                elif k == 'skid':
                    frontwheelradpersec = self.d['wheelSpinVel'][0]
                    skid = 0
                    if frontwheelradpersec:
                        skid = (.5555555555 * self.d['speedX'] /
                                frontwheelradpersec - .66124)
                    strout = bargraph(skid, -.05, .4, 50, '*')
                elif k == 'slip':
                    frontwheelradpersec = self.d['wheelSpinVel'][0]
                    slip = 0
                    if frontwheelradpersec:
                        slip = ((self.d['wheelSpinVel'][2] + self.d['wheelSpinVel'][3]) -
                                (self.d['wheelSpinVel'][0] + self.d['wheelSpinVel'][1]))
                    strout = bargraph(slip, -5, 150, 50, '@')
                else:
                    strout = str(self.d.get(k, ''))
            out += "%s: %s\n" % (k, strout)
        return out


# ---------------------------------------------------------------------------
# DriverAction
# ---------------------------------------------------------------------------

class DriverAction():
    '''What the driver is intending to do (i.e. send to the server).
    Composes something like this for the server:
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus 0)(meta 0)'''
    def __init__(self):
        self.actionstr = str()
        self.d = {
            'accel':  0.2,
            'brake':  0,
            'clutch': 0,
            'gear':   1,
            'steer':  0,
            'focus':  [-90, -45, 0, 45, 90],
            'meta':   0,
        }

    def clip_to_limits(self):
        """Clamp all action values to their legal ranges before sending."""
        self.d['steer']  = clip(self.d['steer'],  -1, 1)
        self.d['brake']  = clip(self.d['brake'],   0, 1)
        self.d['accel']  = clip(self.d['accel'],   0, 1)
        self.d['clutch'] = clip(self.d['clutch'],  0, 1)
        if self.d['gear'] not in [-1, 0, 1, 2, 3, 4, 5, 6]:
            self.d['gear'] = 0
        if self.d['meta'] not in [0, 1]:
            self.d['meta'] = 0
        if (type(self.d['focus']) is not list or
                min(self.d['focus']) < -180 or
                max(self.d['focus']) > 180):
            self.d['focus'] = 0

    def __repr__(self):
        self.clip_to_limits()
        out = str()
        for k in self.d:
            out += '(' + k + ' '
            v = self.d[k]
            if type(v) is not list:
                out += '%.3f' % v
            else:
                out += ' '.join([str(x) for x in v])
            out += ')'
        return out

    def fancyout(self):
        '''Specialty output for useful monitoring of bot effectors.'''
        out = str()
        od  = self.d.copy()
        od.pop('gear',  '')
        od.pop('meta',  '')
        od.pop('focus', '')
        for k in sorted(od):
            if k in ('clutch', 'brake', 'accel'):
                strout = '%6.3f %s' % (od[k], bargraph(od[k], 0, 1, 50, k[0].upper()))
            elif k == 'steer':
                strout = '%6.3f %s' % (od[k], bargraph(od[k] * -1, -1, 1, 50, 'S'))
            else:
                strout = str(od[k])
            out += "%s: %s\n" % (k, strout)
        return out


# ---------------------------------------------------------------------------
# Misc utility
# ---------------------------------------------------------------------------

def destringify(s):
    '''Convert a string (or list of strings) to float(s) where possible.'''
    if not s: return s
    if type(s) is str:
        try:
            return float(s)
        except ValueError:
            print("Could not find a value in %s" % s)
            return s
    elif type(s) is list:
        if len(s) < 2:
            return destringify(s[0])
        else:
            return [destringify(i) for i in s]


# ---------------------------------------------------------------------------
# Drive function  (v3.2)
# ---------------------------------------------------------------------------

# RPM thresholds for gear changes
_UPSHIFT_RPM   = 8500   # shift up when RPM exceeds this
# v3.1 fix 2: raised from 4500 to 5500 so the car downshifts earlier on corner
# exits and keeps the engine in its power band from 65-90 km/h.
# If the car hunts between gears on the straight, drop back to 5000 or 4500.
_DOWNSHIFT_RPM = 5500

# ---------------------------------------------------------------------------
# Braking curve
# ---------------------------------------------------------------------------
# v3.2 change 2: raised 13.0 -> 14.0  (ABS prevents lock-up; braking point ~5-8 m later)
# Revert to 13.0 if the car runs wide on corner entry.  Try 15.0 if 14.0 is clean.
_DECEL = 14.0

# Maximum/minimum speed targets (km/h)
_MAX_SPEED = 250.0
_MIN_SPEED =  50.0

# Proportional gain for throttle controller
_ACCEL_GAIN = 0.08

# Brake gain — applied when speed exceeds target by more than dead-band
_BRAKE_GAIN     = 0.12
_BRAKE_DEADBAND =  3.0

# ---------------------------------------------------------------------------
# ABS — Anti-lock Braking System
# ---------------------------------------------------------------------------
_WHEEL_RADIUS = 0.3179   # metres — typical TORCS default car


def _abs_brake(brake, speed_kmh, wheel_spin):
    '''Reduce brake pressure if any wheel is locking up.

    Compares car speed (m/s) to mean wheel surface speed.  Halves brake
    pressure when wheels are >2 m/s slower than the car.  Skipped below
    3 km/h to avoid false triggers at standstill.
    '''
    v = speed_kmh / 3.6
    if v < 3.0:
        return brake
    wheel_v = (sum(wheel_spin) / 4.0) * _WHEEL_RADIUS
    if v - wheel_v > 2.0:
        brake *= 0.5
    return brake


# ---------------------------------------------------------------------------
# Steering
# ---------------------------------------------------------------------------
# steer = angle / _STEER_DIVISOR
# 0.5 = 2.0 per radian  (default; good for 150 km/h)
# 0.7 = 1.4 per radian  (if the car zig-zags on straights)
# 0.4 = 2.5 per radian  (if the car understeers in corners)
_STEER_DIVISOR = 0.5

_STEER_DERIV_GAIN  = 0.2

# Speed-variable track-centre spring.
#   k_pos = BASE + EXTRA * clip((120 - speed) / 60, 0, 1)
#   At speed <= 60 km/h: k_pos = BASE + EXTRA
#   At speed >= 120 km/h: k_pos = BASE  (unchanged from v3)
# v3.2 change 1: _STEER_TRACK_EXTRA raised 0.40 -> 0.50
#   k_pos at <=60 km/h: 0.80  (was 0.70 in v3.1, 0.30 in v3)
#   Target: |pos| < 0.5 at corner exit.  Revert to 0.40 if the car weaves.
_STEER_TRACK_BASE  = 0.30   # high-speed gain (>=120 km/h)
_STEER_TRACK_EXTRA = 0.50   # v3.2: raised from 0.40; low-speed stiffening

# Corner-anticipation gain.
# Sign note: TORCS negative angles point RIGHT, positive angles point LEFT.
# left_far / right_far are therefore physically swapped vs. their names.
# The formula (right_far - left_far) is also swapped, and the errors cancel:
#   road opens LEFT  -> bias > 0 -> positive steer -> steers left
#   road opens RIGHT -> bias < 0 -> negative steer -> steers right
# Do NOT fix the variable names or the formula independently.
_STEER_CORNER_GAIN = 0.12

# Wide-angle beam indices for corner anticipation
_CORNER_LEFT_INDICES  = [0, 1]    # sensor angles -45 deg, -19 deg (physically RIGHT)
_CORNER_RIGHT_INDICES = [17, 18]  # sensor angles +19 deg, +45 deg (physically LEFT)

# TCS — rear spin excess threshold scales with speed
_TCS_BASE_THRESHOLD = 3.0
_TCS_ACCEL_CUT      = 0.15

# Module-level state — persists across steps within an episode
_prev_steer = 0.0
_tick       = 0


def _corner_speed_target(track_sensors):
    '''Compute a safe target speed using a physics-based stopping-distance curve.

    Uses the LONGEST reading across seven centre beams (indices 6-12, approx +-1.7 deg).
    Taking max looks where the road goes (apex line), not at the nearest wall.

    TORCS sends -1.0 for beams that hit nothing (off-road / sensor out of range).
    forward is clamped to 0 before sqrt to prevent ValueError.

    Formula:  target_km/h = 3.6 * sqrt(2 * _DECEL * forward_metres)

    Examples at _DECEL = 14 m/s^2:
        200 m straight -> target approx 267 km/h  (capped at _MAX_SPEED = 250)
         80 m approach -> target approx 169 km/h
         30 m hairpin  -> target approx 103 km/h
         10 m wall     -> target approx  59 km/h  (approx _MIN_SPEED = 50)
    '''
    forward = max(track_sensors[i] for i in (6, 7, 8, 9, 10, 11, 12))
    forward = max(forward, 0.0)   # guard against -1.0 off-road readings
    target  = 3.6 * math.sqrt(2 * _DECEL * forward)
    return clip(target, _MIN_SPEED, _MAX_SPEED)


def _corner_geometry(track_sensors):
    '''Return a signed corner-anticipation bias from the wide-angle beams.

    Beam layout (from init string "-45 -19 ... +19 +45"):
      index 0  = -45 deg,  index 1  = -19 deg  -> stored in left_far
      index 17 = +19 deg,  index 18 = +45 deg  -> stored in right_far

    NOTE on sign: TORCS negative angles point RIGHT, positive point LEFT.
    left_far / right_far are therefore physically backwards.  The formula
    (right_far - left_far) / total is also backwards, and the two errors cancel:
      road opens LEFT  -> bias > 0 -> positive steer -> steers left
      road opens RIGHT -> bias < 0 -> negative steer -> steers right
    Do not change either the variable names or the formula independently.
    '''
    left_far  = sum(track_sensors[i] for i in _CORNER_LEFT_INDICES)
    right_far = sum(track_sensors[i] for i in _CORNER_RIGHT_INDICES)
    total     = left_far + right_far
    if total < 1.0:
        return 0.0
    return (right_far - left_far) / total


def _rpm_gear(current_gear, rpm):
    '''Return the correct gear based on RPM.
    Shifts up at high RPM, down at low RPM.  Clamped to [1, 6].
    '''
    gear = current_gear
    if rpm > _UPSHIFT_RPM and gear < 6:
        gear += 1
    elif rpm < _DOWNSHIFT_RPM and gear > 1:
        gear -= 1
    return gear


def drive_example(c):
    '''v3.2 driver -- one-change-per-lap tuning pass.

    v3.2 changes vs v3.1:
      1. _STEER_TRACK_EXTRA raised 0.40 -> 0.50
         k_pos at <=60 km/h is now 0.80 (was 0.70).
         Goal: |pos| < 0.5 at corner exit; higher fwd; higher tgt.
         Revert to 0.40 if the car weaves at low speed.

      2. _DECEL raised 13.0 -> 14.0 m/s^2
         ABS prevents lock-up, so the car can brake later.
         Braking point moves ~5-8 m closer to each apex.
         Revert to 13.0 if the car runs wide on corner entry.

    Inherited from v3.1 (unchanged):
      - _DOWNSHIFT_RPM = 5500 (earlier downshift on corner exits)
      - speed-variable k_pos spring (stiffer at low speed)
      - ABS on all braking
      - extended CORNER telemetry (gear, rpm, acc columns)
    '''
    global _prev_steer, _tick

    S = c.S.d
    R = c.R.d

    speed    = S['speedX']          # km/h, forward velocity
    angle    = S['angle']           # radians, heading error
    trackPos = S['trackPos']        # lateral position, -1..1
    rpm      = S.get('rpm', 3000)
    gear     = int(S.get('gear', 1))

    track_sensors = S.get('track', [100.0] * 19)
    if not isinstance(track_sensors, list):
        track_sensors = [100.0] * 19

    # ------------------------------------------------------------------
    # 1. CORNER-AWARE TARGET SPEED
    # ------------------------------------------------------------------
    target_speed = _corner_speed_target(track_sensors)

    # ------------------------------------------------------------------
    # 2. STEERING -- angle / divisor  +  speed-variable recovery  +  corner bias
    # ------------------------------------------------------------------
    steer_angle = angle / _STEER_DIVISOR

    # v3.1 fix 1: speed-variable track-centre spring.
    # Ramps from k_pos=0.70 at <=60 km/h down to k_pos=0.30 at >=120 km/h.
    # Stiffer at low speed keeps the car centred through tight corners so the
    # forward beams see further and tgt stays higher.
    ramp           = clip((120.0 - speed) / 60.0, 0.0, 1.0)
    k_pos          = _STEER_TRACK_BASE + _STEER_TRACK_EXTRA * ramp
    steer_recovery = -trackPos * k_pos

    corner_bias    = _corner_geometry(track_sensors) * _STEER_CORNER_GAIN

    raw_steer   = steer_angle + steer_recovery + corner_bias
    steer_deriv = _STEER_DERIV_GAIN * (raw_steer - _prev_steer)
    R['steer']  = clip(raw_steer - steer_deriv, -1, 1)
    _prev_steer = R['steer']

    # ------------------------------------------------------------------
    # 3. THROTTLE & BRAKE
    # ------------------------------------------------------------------
    speed_error = target_speed - speed

    if speed_error > 0:
        R['accel'] = clip(speed_error * _ACCEL_GAIN, 0.0, 1.0)
        R['brake'] = 0.0
    else:
        R['accel'] = 0.0
        overshoot  = -speed_error
        if overshoot > _BRAKE_DEADBAND:
            R['brake'] = clip((overshoot - _BRAKE_DEADBAND) * _BRAKE_GAIN,
                              0.0, 1.0)
        else:
            R['brake'] = 0.0

    # Low-speed kick: ensure the car always gets moving from a standstill
    if speed < 5.0:
        R['accel'] = max(R['accel'], 0.5)
        R['brake'] = 0.0

    # ------------------------------------------------------------------
    # 4. ABS -- halve brake if any wheel is locking up
    # ------------------------------------------------------------------
    R['brake'] = _abs_brake(R['brake'], speed, S['wheelSpinVel'])

    # ------------------------------------------------------------------
    # 5. TRACTION CONTROL -- speed-scaled threshold
    # ------------------------------------------------------------------
    rear_spin  = S['wheelSpinVel'][2] + S['wheelSpinVel'][3]
    front_spin = S['wheelSpinVel'][0] + S['wheelSpinVel'][1]
    tcs_threshold = _TCS_BASE_THRESHOLD + speed * 0.05
    if (rear_spin - front_spin) > tcs_threshold:
        R['accel'] = max(0.0, R['accel'] - _TCS_ACCEL_CUT)

    # ------------------------------------------------------------------
    # 6. GEAR -- RPM-based automatic transmission
    # ------------------------------------------------------------------
    R['gear'] = _rpm_gear(gear, rpm)

    # ------------------------------------------------------------------
    # 7. TELEMETRY
    #
    # Corner log: fires when speed < 120 OR tgt < 120, every 3rd step.
    # This covers braking on approach as well as apex and exit.
    #
    # Columns to watch:
    #   pos  -- should stay below |0.5| if fix 1 is working
    #   fwd  -- should be 20+ m in corners once pos is controlled
    #   gear -- if high with low rpm on exit, fix 2 (_DOWNSHIFT_RPM=5500) is needed
    #   acc  -- if < 1.0 while speed < tgt, TCS is cutting throttle
    #
    # Straight log: once per 25 steps (~2 Hz) otherwise.
    # ------------------------------------------------------------------
    _tick += 1
    if (speed < 120 or target_speed < 120) and _tick % 3 == 0:
        print("CORNER spd %.0f tgt %.0f fwd %.0f pos %.2f gear %d rpm %.0f acc %.2f" % (
            speed,
            target_speed,
            max(track_sensors[6:13]),
            trackPos,
            R['gear'],
            rpm,
            R['accel'],
        ))
    elif _tick % 25 == 0:
        print("dist %5.0f  spd %3.0f  tgt %3.0f  gear %d" % (
            S.get('distFromStart', 0),
            speed,
            target_speed,
            R['gear'],
        ))


# ---------------------------------------------------------------------------
# MAIN -- standalone client loop
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    #### DO NOT HARDCODE THE PORT ####
    C = Client()
    for step in range(C.maxSteps, 0, -1):
        C.get_servers_input()
        drive_example(C)
        C.respond_to_server()
    C.shutdown()
