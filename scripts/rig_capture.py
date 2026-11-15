"""
Automated parametric captures with the servo rig (hardware/servo_rig).

Set the project up in ``nam-capture`` first: input WAVs, audio I/O, loopback, route test,
knobs and plan. Name the knobs exactly as the rig does (rig.KNOBS). Hand-flipped switches
are knobs with min 0, max 1, step 1. Then capture one switch state at a time:

    python scripts/rig_capture.py status PROJECT_DIR
    python scripts/rig_capture.py run PROJECT_DIR --switch "High Mid Gain=0" --switch "Bright=0"

``run`` captures every pending entry planned at that switch state, then stops, so the
switches only get flipped between runs. It's safe to stop at any point: each capture is
saved as it completes, and the next run resumes from the pending entries.
"""

import argparse
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hardware/servo_rig/firmware"))
import rig as _rig  # noqa: E402
from nam.capture.project import load_project  # noqa: E402
from nam.capture.session import CaptureSession, CaptureSessionError  # noqa: E402

# Let the pots, servos and amp settle after a move, with the servo pulses off.
SETTLE_S = 0.5


def _split_knobs(project):
    servo, manual = [], []
    for knob in project.knobs:
        if knob.name in _rig.KNOBS:
            servo.append(knob.name)
        elif knob.min == 0 and knob.max == 1 and knob.step == 1:
            manual.append(knob.name)
        else:
            raise SystemExit(
                f"Knob {knob.name!r} is neither a rig knob {_rig.KNOBS} nor a 0/1 switch"
            )
    return servo, manual


def _state(entry, manual):
    return tuple(int(round(entry.params[name])) for name in manual)


def _describe(state, manual):
    return ", ".join(f"{n}={'on' if v else 'off'}" for n, v in zip(manual, state))


def status(project_dir):
    project = load_project(project_dir)
    servo, manual = _split_knobs(project)
    calibration = _rig.load_calibration()
    missing = [k for k in servo if len(calibration.get(k, [])) < 2]
    print(f"Servo knobs: {', '.join(servo)}")
    print(f"Calibration missing for: {', '.join(missing) or 'none'}")
    pending = Counter(_state(e, manual) for e in project.pending_entries())
    done = len(project.captured_entries())
    print(f"Captured {done}, pending {sum(pending.values())}")
    for state, count in sorted(pending.items()):
        print(f"  {count:4d} pending at {_describe(state, manual) or 'no switches'}")


def run(project_dir, switches, limit):
    project = load_project(project_dir)
    servo, manual = _split_knobs(project)
    if set(switches) != set(manual):
        raise SystemExit(f"Give --switch for exactly these: {manual}")
    state = tuple(switches[name] for name in manual)
    entries = [e for e in project.pending_entries() if _state(e, manual) == state]
    entries = entries[:limit] if limit else entries
    if not entries:
        return print(f"Nothing pending at {_describe(state, manual)}")

    calibration = _rig.load_calibration()
    for name in servo:  # fail before moving anything
        _rig.dial_to_us(calibration, name, 5.0)
    rig = _rig.Rig()
    session = CaptureSession(project, Path(project_dir))
    session.load_inputs()
    print(f"Capturing {len(entries)} at {_describe(state, manual)}", flush=True)
    flagged = 0
    try:
        for n, entry in enumerate(entries, 1):
            rig.set({name: entry.params[name] for name in servo}, calibration)
            rig.off()
            time.sleep(SETTLE_S)
            try:
                qa = session.capture_entry(entry)
            except CaptureSessionError as exc:
                raise SystemExit(f"Stopped at entry {entry.index}: {exc}")
            flags = [m for m in qa.messages] + (["clipping"] if qa.clipping else [])
            flagged += bool(flags)
            settings = " ".join(f"{k}={v:g}" for k, v in entry.params.items())
            print(
                f"[{n}/{len(entries)}] #{entry.index} {settings} "
                f"peak={qa.peak:.3f} {'FLAGS: ' + '; '.join(flags) if flags else 'ok'}",
                flush=True,
            )
    finally:
        rig.off()
        session.close_audio()
    print(f"Done: {len(entries)} captured, {flagged} with QA flags")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status").add_argument("project_dir", type=Path)
    p = sub.add_parser("run")
    p.add_argument("project_dir", type=Path)
    p.add_argument("--switch", action="append", default=[], help='"Name=0" or "Name=1"')
    p.add_argument("--limit", type=int, default=0, help="stop after N captures")
    args = parser.parse_args()
    if args.cmd == "status":
        status(args.project_dir)
    else:
        switches = {k.strip(): int(v) for k, v in (s.split("=") for s in args.switch)}
        run(args.project_dir, switches, args.limit)


if __name__ == "__main__":
    main()
