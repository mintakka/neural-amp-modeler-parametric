"""
Pot gear rev7: the user's rev6 pot gear (step/source/pot_gear_rev6.step) with its 20 teeth
replaced by the 15-tooth, profile-shifted teeth of the 41:15 (2.73:1) pair in servo_rig.py.

Everything else is kept exactly as rev6 has it: the plain section that fits through the
carrier, the D-bore, the set-screw nut trap, and the inner circular cutout at the amp-face
end. Teeth span the same length as rev6 (z 5.85 to the amp-face end).

    python pot_gear_rev7.py      # writes step/pot_gear_rev7.step
"""

from pathlib import Path

import cadquery as cq

from servo_rig import BACKLASH, MODULE, SHIFT_POT, Z_POT, involute_gear

HERE = Path(__file__).parent
REV6 = HERE / "step/source/pot_gear_rev6.step"
OUT = HERE / "step/pot_gear_rev7.step"
TEETH_START = 5.85   # MEASURED from rev6: teeth run from here to the amp-face end
CORE_R = 8.0         # inside rev6's tooth roots (8.75): every internal feature is within this


def main():
    rev6 = cq.importers.importStep(str(REV6)).solids().vals()[0]
    bb = rev6.BoundingBox()
    cx, cy = (bb.xmin + bb.xmax) / 2, (bb.ymin + bb.ymax) / 2
    length = bb.zmax
    teeth_len = length - TEETH_START

    def slab(z0, z1, r=50.0):
        return cq.Workplane("XY").workplane(offset=z0).center(cx, cy).circle(r).extrude(z1 - z0)

    # Plain section (fits through the carrier), exactly as rev6.
    plain = cq.Workplane("XY").add(rev6).intersect(slab(0, TEETH_START))
    # Rev6's internal features in the toothed length: bore, nut trap, inner cutout.
    cavities = slab(TEETH_START, length, CORE_R).cut(cq.Workplane("XY").add(rev6))
    teeth = (involute_gear(Z_POT, MODULE, teeth_len, BACKLASH, SHIFT_POT)
             .translate((cx, cy, TEETH_START)))
    gear = plain.union(teeth.cut(cavities))

    solids = gear.solids().vals()
    print(f"rev6 axis ({cx:.3f}, {cy:.3f}), length {length:.2f} mm, teeth {teeth_len:.2f} mm")
    print(f"solids: {len(solids)}, volume {sum(s.Volume() for s in solids):.0f} mm^3")
    if len(solids) != 1:
        raise SystemExit("FAIL: result is not one solid")
    cq.exporters.export(gear, str(OUT))
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
