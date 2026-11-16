"""
Host side of the servo rig: talk to the Pico 2 and map dial numbers (0-10) to servo pulses.

    python rig.py ping
    python rig.py at Presence 800            # after power-up: where the servo physically is
    python rig.py speed 150                  # ramp speed in us/s
    python rig.py raw Presence 1500          # move by pulse width (calibration)
    python rig.py cal Presence 1500 5.2      # record: at 1500 us the pointer reads 5.2
    python rig.py set Presence=5 Bass=7.5    # move by dial value (needs >= 2 cal points)
    python rig.py off                        # stop pulses before recording
    python rig.py show

Calibration lives in rig_calibration.json next to this file.
"""

import glob
import json
import sys
from pathlib import Path

import serial

KNOBS = (
    "Presence",
    "Bass",
    "Middle",
    "Treble",
    "Master Volume",
    "Treble Volume",
    "Bass Volume",
)
# Servo mounting (numbered 1-7 left to right, matching KNOBS and GP0-GP6):
#   bottom row: servos 1, 3, 5, 7 -> Presence, Middle, Master Volume, Bass Volume
#   top row:    servos 2, 4, 6    -> Bass, Treble, Treble Volume   (mounted upside down)
# Every pot on the amp turns the same way, but the top-row servos face the opposite way,
# so the same pulse change turns their pots in the opposite direction and their "pot at 0"
# end is at the opposite end of the servo's travel.
#
# Everything in this file is in KNOB SPACE: a low pulse width means "pot toward 0" and a
# high one "pot toward 10", for all seven knobs. to_servo() mirrors the top-row servos
# around 1500 us (us -> 3000 - us) just before a command goes to the firmware; nothing else
# needs to know which row a servo is in. The firmware itself only ever sees raw servo pulses.
INVERTED = frozenset({"Bass", "Treble", "Treble Volume"})  # servos 2, 4, 6 (top row)
CAL_PATH = Path(__file__).with_name("rig_calibration.json")


def to_servo(knob, us):
    """Knob-space pulse width -> the pulse the firmware should send to that servo."""
    us = int(round(us))
    return 3000 - us if knob in INVERTED else us


class Rig:
    def __init__(self, port=None, timeout=60.0):
        if port is None:
            ports = sorted(glob.glob("/dev/cu.usbmodem*"))
            if not ports:
                raise RuntimeError("No Pico found on /dev/cu.usbmodem*")
            port = ports[0]
        self._serial = serial.Serial(port, 115200, timeout=timeout)
        self._serial.reset_input_buffer()

    def command(self, line):
        self._serial.write((line + "\n").encode())
        reply = self._serial.readline().decode().strip()
        if not reply.startswith("OK"):
            raise RuntimeError(f"{line!r} -> {reply or 'timeout'}")
        return reply[2:].strip()

    def at(self, knob, us):
        self.command(f"AT {KNOBS.index(knob)} {to_servo(knob, us)}")

    def raw(self, knob, us):
        self.command(f"SET {KNOBS.index(knob)} {to_servo(knob, us)}")

    def set(self, values, calibration):
        targets = ["-"] * len(KNOBS)
        for knob, dial in values.items():
            targets[KNOBS.index(knob)] = str(to_servo(knob, dial_to_us(calibration, knob, dial)))
        self.command("MOVE " + " ".join(targets))

    def off(self):
        self.command("OFF")


def load_calibration():
    return json.loads(CAL_PATH.read_text()) if CAL_PATH.exists() else {}


def dial_to_us(calibration, knob, dial):
    """Piecewise-linear through the knob's (us, dial) points; extrapolates at the ends."""
    points = sorted((d, u) for u, d in calibration.get(knob, []))
    if len(points) < 2:
        raise ValueError(f"{knob} needs at least 2 calibration points")
    for (d0, u0), (d1, u1) in zip(points, points[1:]):
        if dial <= d1 or (d1, u1) == points[-1]:
            return round(u0 + (dial - d0) * (u1 - u0) / (d1 - d0))


def main(argv):
    if not argv:
        sys.exit(__doc__)
    cmd, args = argv[0], argv[1:]
    calibration = load_calibration()
    if cmd == "cal":
        knob, us, dial = args[0], int(args[1]), float(args[2])
        KNOBS.index(knob)
        points = [p for p in calibration.get(knob, []) if p[0] != us] + [[us, dial]]
        calibration[knob] = sorted(points)
        CAL_PATH.write_text(json.dumps(calibration, indent=2))
        return print(f"{knob}: {calibration[knob]}")
    if cmd == "show":
        return print(json.dumps(calibration, indent=2))
    rig = Rig()
    if cmd == "ping":
        print(rig.command("PING"))
    elif cmd == "at":
        rig.at(" ".join(args[:-1]), int(args[-1]))
    elif cmd == "speed":
        rig.command(f"SPEED {float(args[0])}")
    elif cmd == "raw":
        rig.raw(" ".join(args[:-1]), int(args[-1]))
    elif cmd == "set":
        values = dict(a.split("=") for a in args)
        rig.set({k: float(v) for k, v in values.items()}, calibration)
    elif cmd == "off":
        rig.off()
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
