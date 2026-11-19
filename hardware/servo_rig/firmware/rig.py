"""
Host side of the servo rig: talk to the Pico and map dial numbers (0-10) to servo pulses.

    python rig.py ping
    python rig.py get                        # known positions (0 = unknown)
    python rig.py home Middle                # re-home, GEAR UNMESHED: hold at 1500 us
    python rig.py raw Middle 1450            # one smooth move to a pulse width
    python rig.py cal Middle 1450 4.5        # record: at 1450 us the pointer reads 4.5
    python rig.py limits Middle 1060 1930    # firmware-enforced safe range for that servo
    python rig.py set Middle=5 Bass=7.5      # move by dial value (from calibration)
    python rig.py off                        # stop pulses (before recording)
    python rig.py show                       # calibration and limits

Every pulse width here is the RAW pulse sent to that servo (1000-2000 us); nothing is
mirrored. Calibration maps each knob's raw pulses to dial numbers, so a knob whose dial
goes down as the pulse goes up simply has a negative slope. Calibration lives in
rig_calibration.json next to this file; limits are also stored in the Pico's flash.
"""

import glob
import itertools
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
# Measured on the 41:15 gear rig: on ALL seven knobs a rising pulse turns the pot
# counterclockwise (dial down), top row included, at about 107 us per clock hour.
# Calibration records which pulse gives which dial reading per knob, so nothing mirrors pulses.
TOP_ROW = frozenset({"Bass", "Treble", "Treble Volume"})  # servos 2, 4, 6
HOME_US = 1500  # servo centre: re-home here with the gear unmeshed, mesh with the pot at 5
CAL_PATH = Path(__file__).with_name("rig_calibration.json")


class Rig:
    def __init__(self, port=None, timeout=120.0):
        if port is None:
            ports = sorted(glob.glob("/dev/cu.usbmodem*"))
            if not ports:
                raise RuntimeError("No Pico found on /dev/cu.usbmodem*")
            port = ports[0]
        self._serial = serial.Serial(port, 115200, timeout=timeout)
        self._serial.reset_input_buffer()
        self._tags = itertools.count(1)

    def command(self, line):
        """Send one command and return its reply, skipping stale replies to old commands."""
        tag = f"@{next(self._tags)}"
        self._serial.write(f"{line} {tag}\n".encode())
        while True:
            reply = self._serial.readline().decode().strip()
            if not reply:
                raise RuntimeError(f"{line!r} -> timeout")
            if reply.endswith(tag):
                break
        reply = reply[: -len(tag)].strip()
        if not reply.startswith("OK"):
            raise RuntimeError(f"{line!r} -> {reply}")
        return reply[2:].strip()

    def home(self, knob):
        self.command(f"AT {KNOBS.index(knob)} {HOME_US}")

    def raw(self, knob, us):
        self.command(f"SET {KNOBS.index(knob)} {int(round(us))}")

    def limits(self, knob, lo, hi):
        self.command(f"LIMITS {KNOBS.index(knob)} {int(lo)} {int(hi)}")

    def set(self, values, calibration):
        targets = ["-"] * len(KNOBS)
        for knob, dial in values.items():
            targets[KNOBS.index(knob)] = str(dial_to_us(calibration, knob, dial))
        self.command("MOVE " + " ".join(targets))

    def off(self):
        self.command("OFF")


def load_calibration():
    return json.loads(CAL_PATH.read_text()) if CAL_PATH.exists() else {}


def dial_to_us(calibration, knob, dial, margin=0.3):
    """Piecewise-linear through the knob's (us, dial) points. Refuses dials more than
    `margin` beyond the calibrated span instead of extrapolating toward a pot stop."""
    points = sorted((d, u) for u, d in calibration.get(knob, []))
    if len(points) < 2:
        raise ValueError(f"{knob} needs at least 2 calibration points")
    if not points[0][0] - margin <= dial <= points[-1][0] + margin:
        raise ValueError(
            f"{knob}={dial} is outside its calibrated span "
            f"{points[0][0]:g}-{points[-1][0]:g}; calibrate closer to that end first"
        )
    for (d0, u0), (d1, u1) in zip(points, points[1:]):
        if dial <= d1 or (d1, u1) == points[-1]:
            return round(u0 + (dial - d0) * (u1 - u0) / (d1 - d0))


def main(argv):
    if not argv:
        sys.exit(__doc__)
    cmd, args = argv[0], argv[1:]
    calibration = load_calibration()
    if cmd == "cal":
        knob, us, dial = " ".join(args[:-2]), int(args[-2]), float(args[-1])
        KNOBS.index(knob)
        points = [p for p in calibration.get(knob, []) if p[0] != us] + [[us, dial]]
        calibration[knob] = sorted(points)
        CAL_PATH.write_text(json.dumps(calibration, indent=2))
        return print(f"{knob}: {calibration[knob]}")
    if cmd == "show":
        return print(json.dumps(calibration, indent=2))
    rig = Rig()
    knob = " ".join(args[:-1]) if args else ""
    if cmd == "ping":
        print(rig.command("PING"))
    elif cmd == "get":
        print(dict(zip(KNOBS, rig.command("GET").split())))
    elif cmd == "home":
        rig.home(" ".join(args))
    elif cmd == "raw":
        rig.raw(knob, int(args[-1]))
    elif cmd == "limits":
        rig.limits(" ".join(args[:-2]), int(args[-2]), int(args[-1]))
    elif cmd == "speed":
        rig.command(f"SPEED {float(args[0])}")
    elif cmd == "set":
        values = dict(a.split("=") for a in args)
        rig.set({k: float(v) for k, v in values.items()}, calibration)
    elif cmd == "off":
        rig.off()
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
