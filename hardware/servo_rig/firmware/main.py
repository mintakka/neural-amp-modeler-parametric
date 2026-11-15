# Servo rig firmware for a Raspberry Pi Pico 2 (MicroPython).
# Seven SG92R servos on GP0..GP6, driven over USB serial with line commands:
#   PING                  -> OK servo-rig 1
#   SET <i> <us>          move servo i (0-6) to a pulse width, approaching from below
#   MOVE <us0> ... <us6>  move all seven (use - to leave one where it is)
#   OFF [i]               stop pulses (one servo or all) so they don't buzz or hum
#   GET                   -> OK <us0> ... <us6> (0 = off)
# Every command answers one line: "OK ..." when done (motion settled) or "ERR ...".
import sys
import time
from machine import PWM, Pin

PINS = (0, 1, 2, 3, 4, 5, 6)
US_MIN, US_MAX = 500, 2500
# Approach every target from below by this much so gear slack always sits on the same side.
OVERSHOOT_US = 60
# SG92R: about 0.1 s per 60 degrees, roughly 0.15 ms per microsecond of pulse change.
SETTLE_MS_PER_US = 0.2
SETTLE_MS_MIN = 150
PERIOD_US = 20000

servos = []
for p in PINS:
    pwm = PWM(Pin(p))
    pwm.freq(50)
    pwm.duty_u16(0)
    servos.append(pwm)
current = [0] * len(PINS)


def _write(i, us):
    servos[i].duty_u16(int(us * 65535 // PERIOD_US))
    current[i] = us


def _settle(distance_us):
    time.sleep_ms(max(SETTLE_MS_MIN, int(distance_us * SETTLE_MS_PER_US)))


def move(targets):
    """targets: {servo: us}. Moves together, each finishing on an upward approach."""
    for i, us in targets.items():
        if not (0 <= i < len(PINS)) or not (US_MIN <= us <= US_MAX):
            raise ValueError("servo %d to %d us out of range" % (i, us))
    # Phase 1: anything that must come down (or is off) goes below its target.
    low = {}
    for i, us in targets.items():
        if current[i] == 0 or current[i] >= us:
            low[i] = max(US_MIN, us - OVERSHOOT_US)
    if low:
        dist = max(abs((current[i] or US_MAX) - us) for i, us in low.items())
        for i, us in low.items():
            _write(i, us)
        _settle(dist)
    # Phase 2: everything rises onto its target.
    dist = max(abs(current[i] - us) for i, us in targets.items())
    for i, us in targets.items():
        _write(i, us)
    _settle(dist)


def off(i=None):
    for j in range(len(PINS)) if i is None else (i,):
        servos[j].duty_u16(0)
        current[j] = 0


def handle(line):
    parts = line.split()
    if not parts:
        return None
    cmd = parts[0].upper()
    if cmd == "PING":
        return "OK servo-rig 1"
    if cmd == "SET" and len(parts) == 3:
        move({int(parts[1]): int(parts[2])})
        return "OK"
    if cmd == "MOVE" and len(parts) == 1 + len(PINS):
        move({i: int(v) for i, v in enumerate(parts[1:]) if v != "-"})
        return "OK"
    if cmd == "OFF" and len(parts) <= 2:
        off(int(parts[1]) if len(parts) == 2 else None)
        return "OK"
    if cmd == "GET":
        return "OK " + " ".join(str(us) for us in current)
    return "ERR unknown command: " + line


while True:
    line = sys.stdin.readline().strip()
    try:
        reply = handle(line)
    except Exception as e:
        reply = "ERR " + str(e)
    if reply is not None:
        print(reply)
