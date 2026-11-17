#!/usr/bin/env python
"""Parametric servo rig that turns the 7 knob pots of a PRS HDRX20 chassis.

Run:  python servo_rig.py
Writes one STEP per unique printable part (in print orientation, on the bed at
the origin) to ./step/, plus step/assembly.step with everything in place
(panel, posts and servos are placeholder bodies, assembly only).

Coordinate system (assembly): X along the panel (0 = panel left edge),
Y up (0 = panel bottom edge), Z out of the panel toward the user (0 = face).

Mechanism: each pot gets a "pair" = servo + carrier + servo gear + pot gear.
The carrier holds the servo and has a bearing ring that rides on the pot
gear's hub, so the gear center distance is fixed by the carrier itself.
Each carrier bolts to the frame through crossed slots (carrier slots run in Y,
frame slots run in X), so the whole pair can be trimmed +/-ADJUST mm in X and
Y at assembly. The frame clamps onto the chassis with C-jaws behind the face.
"""
import math
import os
import sys
from itertools import combinations

import cadquery as cq

# =============================================================================
# CONFIG  (mm). MEASURED = user caliper/ruler value, ESTIMATED = from a photo
# or a typical datasheet: measure before the final print.
# =============================================================================
# --- Chassis / panel ---------------------------------------------------------
PANEL_W = 406.4          # MEASURED  face width (16")
PANEL_H = 58.7           # MEASURED  face height (2-5/16")
CHASSIS_D = 182.6        # MEASURED  chassis depth (7-3/16")
BEND_R = 3.0             # ESTIMATED radius of the rounded face->top/bottom bends
JAW_REACH = 22.0         # jaws reach this far behind the face (tubes start ~25)

# --- Pot posts (left -> right) -----------------------------------------------
POT_NAMES = ["Presence", "Bass", "Middle", "Treble",
             "Master", "TrebleVol", "BassVol"]
POST_X0 = 90.0           # MEASURED (approx) Presence post center from left edge
POST_PITCH = 38.1        # MEASURED  (Presence->BassVol 228.6 mm, even spacing)
POST_X = [POST_X0 + i * POST_PITCH for i in range(7)]
POST_Y = 25.0            # MEASURED (approx) post centers above bottom edge
POST_D = 6.0             # MEASURED  D-shaft diameter
POST_FLAT = 4.0          # MEASURED  flat to opposite side
POST_LEN = 9.0           # MEASURED  post length beyond the nut (9-10 mm)
NUT_AF = 12.7            # ESTIMATED nut across flats (1/2")
NUT_PROUD = 2.5          # ESTIMATED nut height proud of panel
BUSHING_D = 9.5          # ESTIMATED threaded bushing diameter
BUSHING_PROUD = 4.0      # ESTIMATED bushing end proud of panel (pot gear stop)

# --- Controls that must stay reachable (x, y, keep-out radius) ---------------
# Defaults: HMG midway Treble/Master, Bright midway TrebleVol/BassVol.
TOGGLE_HMG = ((POST_X[3] + POST_X[4]) / 2, POST_Y + 25.0, 10.0)   # ESTIMATED
TOGGLE_BRIGHT = ((POST_X[5] + POST_X[6]) / 2, POST_Y + 25.0, 10.0)  # ESTIMATED
POWER_TOGGLE = (30.0, POST_Y + 5.0, 10.0)   # ESTIMATED
POWER_LED = (50.0, POST_Y + 5.0, 6.0)       # ESTIMATED
INPUT_JACK = (365.0, POST_Y, 12.0)          # ESTIMATED (r covers a plug body)
KEEPOUT_DEPTH = 60.0     # keep-out cylinders extend this far from the face
TOGGLE_WINDOW_R = 12.0   # finger windows in the frame plate

# --- Gears (servo gear big, pot gear small -> step-up) -----------------------
MODULE = 1.0
Z_SERVO = 41           # 41:15 = 2.73:1. Our SG92Rs travel only ~115 deg, so the pot needs more step-up
Z_POT = 15
CENTER_DIST = 28.0       # must equal MODULE*(Z_SERVO+Z_POT)/2 (checked)
# Profile shift (x * MODULE): a 15-tooth pinion is below the 17-tooth undercut limit, so
# the pot gear's teeth are shifted outward and the servo gear's inward by the same amount.
# Equal and opposite shifts keep the 28 mm centre distance (the printed carriers set it).
SHIFT_POT = 0.35
SHIFT_SERVO = -SHIFT_POT
PRESSURE_ANGLE = 20.0
BACKLASH = 0.18          # total circular backlash, split between the 2 gears
GEAR_FACE = 4.0          # face width, both gears share one plane
GEAR_GAP = 0.5           # pot gear back face to bushing end
# Servo gear angle around its post (deg, CCW from +X). -90 = straight below.
# Top servos are swung 30 deg left so the toggles to their right stay clear.
SERVO_ANGLE = [-90, 120, -90, 120, -90, 120, -90]

# --- Pot gear hub / fixing ---------------------------------------------------
D_BORE_CLR = 0.25        # radial clearance on the D-bore (0.15 printed too tight)
HUB_R = 8.0
SETSCREW_D = 3.2         # M3 set screw clearance
M3_NUT_AF, M3_NUT_T = 5.6, 2.6   # nut trap (with clearance)
SETSCREW_ABOVE_FACE = 1.8        # set screw axis above gear front face

# --- Servo: Tower Pro SG92R (ESTIMATED from typical SG90/SG92R drawings) -----
SERVO_L = 23.0           # body length (X of drawing)
SERVO_W = 12.2           # body width
SERVO_H = 22.5           # base to body top (shaft side)
SERVO_TAB_L = 32.3       # length over mounting tabs
SERVO_TAB_Z = 16.0       # base to tab underside
SERVO_TAB_T = 2.5        # tab thickness
SERVO_HOLE_SPACING = 27.8
SERVO_SHAFT_FROM_END = 5.9   # shaft axis from the near body end
SERVO_BOSS_R, SERVO_BOSS_H = 5.9, 3.0
# AS-BUILT: the printed frame and carriers put the servo base this far behind
# the servo gear's servo-side face (old horn stack 31.5 - 1.6). Keep it fixed.
SERVO_BASE_TO_GEAR = 29.9
SERVO_FIT = 0.25             # clearance around body in the carrier

# --- Servo gear: direct drive on the output spline (no horn) -----------------
# TowerPro micro spline: 21T, ~4.8 mm OD (Pololu/makerworld; clones vary +/-0.1+)
SPLINE_TEETH = 21            # ESTIMATED
SPLINE_OD = 4.8              # ESTIMATED shaft spline outside diameter
SPLINE_TOOTH_H = 0.3         # ESTIMATED radial tooth depth
SPLINE_LEN = 3.0             # ESTIMATED splined length = engagement depth
SERVO_BASE_TO_SPLINE_TIP = 29.9  # ESTIMATED base to end of output shaft
SPLINE_FIT = 0.05            # TUNE: radial clearance added to the printed bore
                             # (FDM holes print ~0.1 small -> slight press fit)
SPLINE_TIP_CLR = 0.3         # shaft end to bottom of bore
SPLINE_HUB_D = 9.0
GEAR_CASE_CLR = 0.5          # hub end to servo case top
SCREW_CLR_D = 2.3            # center screw (M2 / 2 mm self-tapper) clearance
SCREW_HEAD_D, SCREW_HEAD_H = 4.0, 1.3   # ESTIMATED stock pan head
SCREW_LEN = 6.0              # center screw length used for the reach check

# --- Carrier (servo + bearing ring on pot hub; slots onto the frame) ---------
CARRIER_T = 4.0
RING_CLR = 0.2           # radial clearance carrier ring <-> pot gear hub
RING_WALL = 3.3
ADJUST = 3.0             # +/- adjustment of each pair relative to the frame
CARRIER_BOLT_DX = 14.0   # carrier bolts at servo axis +/- DX ...
CARRIER_BOLT_DY = 17.0   # ... and DY along the servo body (away from the post)
M3_CLR = 3.4
ZIP_SLOT = (4.5, 2.0)

# --- Frame -------------------------------------------------------------------
FRAME_T = 4.0
WALL_T = 8.0             # roof / floor thickness (lapped 4 + 4 at the joint)
RIB_T = 6.0
CLAMP_RIB_T = 8.0
GEAR_CLR = 1.5           # clearance from gear tips to frame walls
LIP_LEN = 12.0           # jaw lip length down/up the face from the bend
JAW_GAP = 3.0            # bottom jaw to chassis bottom (thumbscrew closes it)
TOP_RIBS = [62.0, 160.0, 239.0, 315.0, 343.0]     # x centers (avoid gears/toggles)
BOTTOM_RIBS = [62.0, 343.0]
CLAMP_RIBS = [116.0, 204.0, 280.0]                # bottom jaws with thumbscrews
FRAME_X_MIN = -9.0       # left end wall (X-stop screw bears on chassis side)
XSTOP_GAP = 3.0
SPLIT_X = 142.5          # plate butt line between the two frame halves
LAP = (131.0, 155.0)     # roof/floor lap range
LAP_BOLTS = [(137.0, -10.0), (149.0, 5.0)]        # (x, z) of M3 bolts along Y
INSERT_D = 4.0           # M3 heat-set insert hole
PICO_CENTER = (27.0, 60.0)   # Raspberry Pi Pico 2 (51 x 21, holes 47 x 11.4, M2)
PICO_HOLE_D = 2.2

# --- Chassis clamp (x3, one per CLAMP_RIB; replaces the bottom thumbscrews) --
# The HDRX20 chassis bottom is open, so the bottom jaws have nothing to push on.
# This L-hook sits on top of each bottom jaw (inside the open bottom), its lip
# reaches up behind the 1 mm front wall, and an M3 screw dropped through its
# Z-slot, through the jaw's thumbscrew hole, into the frame's existing insert
# locks it. Push the hook forward against the wall before tightening: the wall
# is pinched between this lip (inside) and the frame's jaw lip (outside).
CHASSIS_T = 1.0          # MEASURED  sheet thickness of the front wall
CLAMP_W = CLAMP_RIB_T    # width in X (matches the jaw it sits on)
CLAMP_BASE_T = 4.0       # ESTIMATED base plate thickness (sits on the jaw top)
CLAMP_LIP_T = 3.0        # ESTIMATED lip thickness behind the wall
CLAMP_LIP_H = 8.0        # ESTIMATED lip height above the wall's bottom edge
CLAMP_GRIP_CLR = 0.1     # ESTIMATED nominal lip-to-wall gap (slot takes the rest)
CLAMP_SLOT = 1.5         # +/- Z travel of the screw slot (push-to-fit)

BED = (250.0, 250.0, 250.0)  # Bambu P2S 256^3, 250 treated as usable
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "step")

# =============================================================================
# Derived geometry
# =============================================================================
R_SERVO_TIP = MODULE * Z_SERVO / 2 + MODULE
R_POT_TIP = MODULE * Z_POT / 2 + MODULE
Z_GEAR0 = BUSHING_PROUD + GEAR_GAP          # gear plane back (panel side)
Z_GEAR1 = Z_GEAR0 + GEAR_FACE
Z_SERVO_BASE = Z_GEAR1 + SERVO_BASE_TO_GEAR
Z_CASE_TOP = Z_SERVO_BASE - SERVO_H - SERVO_BOSS_H       # boss face
Z_SPLINE_TIP = Z_SERVO_BASE - SERVO_BASE_TO_SPLINE_TIP
Z_SG_HUB_END = Z_CASE_TOP - GEAR_CASE_CLR               # servo gear hub end
Z_SG_BORE_END = Z_SPLINE_TIP - SPLINE_TIP_CLR           # bottom of spline bore
Z_SG_CBORE = Z_GEAR0 + SCREW_HEAD_H + 0.3               # counterbore floor
Z_CARRIER1 = Z_SERVO_BASE - SERVO_TAB_Z - SERVO_TAB_T   # tabs rest here
Z_CARRIER0 = Z_CARRIER1 - CARRIER_T
Z_FRAME1 = Z_CARRIER0
Z_FRAME0 = Z_FRAME1 - FRAME_T
HUB_TOP = Z_CARRIER1 - 0.4
POST_TIP = NUT_PROUD + POST_LEN

PAIRS = []
for i, (px, ang) in enumerate(zip(POST_X, SERVO_ANGLE)):
    a = math.radians(ang)
    sx, sy = px + CENTER_DIST * math.cos(a), POST_Y + CENTER_DIST * math.sin(a)
    sgn = 1 if sy > POST_Y else -1          # servo body extends away from posts
    bc = sy + sgn * (SERVO_L / 2 - SERVO_SHAFT_FROM_END)
    PAIRS.append(dict(i=i, name=POT_NAMES[i], P=(px, POST_Y), S=(sx, sy),
                      ang=ang, sgn=sgn, bc=bc,
                      bolts=[(sx + d, sy + sgn * CARRIER_BOLT_DY)
                             for d in (-CARRIER_BOLT_DX, CARRIER_BOLT_DX)],
                      tabholes=[bc + d * SERVO_HOLE_SPACING / 2 for d in (-1, 1)]))
TOP_Y = max(p["S"][1] for p in PAIRS) + R_SERVO_TIP + GEAR_CLR   # roof bottom
BOT_Y = min(p["S"][1] for p in PAIRS) - R_SERVO_TIP - GEAR_CLR   # floor top
FRAME_X_MAX = max(TOP_RIBS + BOTTOM_RIBS) + RIB_T / 2
PLATE_X_MIN = min(BOTTOM_RIBS) - RIB_T / 2


# =============================================================================
# Helpers
# =============================================================================
def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False) \
        .translate((x0, y0, z0))


def cyl_z(x, y, r, z0, z1):
    return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((x, y, z0))


def cyl_y(x, z, r, y0, y1):
    return cq.Workplane("XZ").circle(r).extrude(-(y1 - y0)).translate((x, y0, z))


def cyl_x(y, z, r, x0, x1):
    return cq.Workplane("YZ").circle(r).extrude(x1 - x0).translate((x0, y, z))


def slot_z(x0, y0, x1, y1, r, z0, z1):
    """Stadium between two centers, extruded along Z."""
    L = math.hypot(x1 - x0, y1 - y0)
    s = cq.Workplane("XY").slot2D(L + 2 * r, 2 * r).extrude(z1 - z0)
    return s.rotate((0, 0, 0), (0, 0, 1), math.degrees(math.atan2(y1 - y0, x1 - x0))) \
        .translate(((x0 + x1) / 2, (y0 + y1) / 2, z0))


def hull2d(pts):
    pts = sorted(set(pts))
    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (p[1] - h[-2][1]) -
                                   (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(p)
        return h
    lo, up = half(pts), half(pts[::-1])
    return lo[:-1] + up[:-1]


def circ_pts(x, y, r, n=32):
    return [(round(x + r * math.cos(2 * math.pi * k / n), 4),
             round(y + r * math.sin(2 * math.pi * k / n), 4)) for k in range(n)]


def poly_z(pts, z0, z1):
    return cq.Workplane("XY").polyline(pts).close().extrude(z1 - z0).translate((0, 0, z0))


def to_print(wp, flip=False):
    """Move a part flat on the bed; flip=True puts its +Z face down."""
    s = wp.val()
    if flip:
        s = s.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180)
    bb = s.BoundingBox()
    return s.translate(cq.Vector(-bb.xmin, -bb.ymin, -bb.zmin))


# =============================================================================
# Involute gear
# =============================================================================
def involute_gear(z, m, face, backlash, shift=0.0, pa_deg=PRESSURE_ANGLE, n=10):
    """Spur gear with profile shift `shift` (x), tooth 0 centered on +X, face z=0..face."""
    pa = math.radians(pa_deg)
    r, rb = m * z / 2, m * z / 2 * math.cos(pa)
    ra, rf = r + m * (1 + shift), r - m * (1.25 - shift)
    inv = lambda a: math.tan(a) - a
    s_pitch = math.pi * m / 2 + 2 * shift * m * math.tan(pa)   # tooth thickness @ pitch
    psi = (s_pitch - backlash / 2) / (2 * r)             # half tooth angle @ pitch
    th = lambda rho: psi + inv(pa) - inv(math.acos(min(1.0, rb / rho)))
    r0 = max(rb, rf)
    radii = [r0 + (ra - r0) * k / n for k in range(n + 1)]
    pts = []
    for k in range(z):
        c = 2 * math.pi * k / z
        P = lambda rho, a: (rho * math.cos(a), rho * math.sin(a))
        pts.append(P(rf, c - th(r0)))
        pts += [P(rho, c - th(rho)) for rho in radii]
        pts += [P(rho, c + th(rho)) for rho in reversed(radii)]
        pts.append(P(rf, c + th(r0)))
        pts.append(P(rf, c + math.pi / z))               # root mid-gap
    return cq.Workplane("XY").polyline(pts).close().extrude(face)


def spline_profile(r_root, r_tip, n, z0, z1):
    """Serrated profile (triangular teeth) between r_tip (inner) and r_root."""
    pts = []
    for k in range(2 * n):
        r, a = (r_root if k % 2 else r_tip), math.pi * k / n
        pts.append((r * math.cos(a), r * math.sin(a)))
    return poly_z(pts, z0, z1)


# =============================================================================
# Parts (local coordinates)
# =============================================================================
def make_servo_gear():
    """Local z=0 is the panel-side (screw) face; hub points at the servo."""
    o = Z_GEAR0
    g = involute_gear(Z_SERVO, MODULE, GEAR_FACE, BACKLASH, SHIFT_SERVO)
    g = g.union(cyl_z(0, 0, SPLINE_HUB_D / 2, 0, Z_SG_HUB_END - o))
    g = g.cut(spline_profile(SPLINE_OD / 2 + SPLINE_FIT,
                             SPLINE_OD / 2 - SPLINE_TOOTH_H + SPLINE_FIT,
                             SPLINE_TEETH, Z_SG_BORE_END - o, Z_SG_HUB_END - o + 1))
    g = g.cut(cyl_z(0, 0, SCREW_CLR_D / 2, -1, Z_SG_BORE_END - o + 0.01))
    g = g.cut(cyl_z(0, 0, SCREW_HEAD_D / 2 + 0.2, -1, Z_SG_CBORE - o))
    return g


def d_profile(r, flat_dist, z0, z1):
    c = cyl_z(0, 0, r, z0, z1)
    return c.cut(box(-r - 1, r + 1, -r - 1, -flat_dist, z0 - 1, z1 + 1))


def make_pot_gear():
    """Local z=0 is the panel-side face. Flat of the D faces -Y (set screw)."""
    g = involute_gear(Z_POT, MODULE, GEAR_FACE, BACKLASH, SHIFT_POT)
    hub_h = HUB_TOP - Z_GEAR0
    # The hub starts behind the teeth: on a 15-tooth gear its radius is past the tooth roots.
    g = g.union(cyl_z(0, 0, HUB_R, GEAR_FACE, hub_h))
    g = g.cut(d_profile(POST_D / 2 + D_BORE_CLR,
                        POST_FLAT - POST_D / 2 + D_BORE_CLR, -1, hub_h + 1))
    zs = GEAR_FACE + SETSCREW_ABOVE_FACE
    g = g.cut(cyl_y(0, zs, SETSCREW_D / 2, -HUB_R - 1, 0))
    yn0 = POST_FLAT - POST_D / 2 + D_BORE_CLR + 0.6          # nut inner face
    g = g.cut(box(-M3_NUT_AF / math.sqrt(3) - 0.1, M3_NUT_AF / math.sqrt(3) + 0.1,
                  -(yn0 + M3_NUT_T), -yn0, zs - M3_NUT_AF / 2 - 0.1, hub_h + 1))
    return g


def carrier_outline(p, grow=0.0):
    sx, sy = p["S"]
    sg, bc = p["sgn"], p["bc"]
    near = sy - sg * (SERVO_SHAFT_FROM_END + (SERVO_TAB_L - SERVO_L) / 2 + 2)
    far = sy + sg * (SERVO_L - SERVO_SHAFT_FROM_END + (SERVO_TAB_L - SERVO_L) / 2 + 2)
    body, main = [], []
    for x in (sx - SERVO_W / 2 - 2, sx + SERVO_W / 2 + 2):
        for y in (near, far):
            body += circ_pts(x, y, grow, 8) if grow else [(x, y)]
    for bx, by in p["bolts"]:
        main += circ_pts(bx, by, 6.0 + grow)
    main += circ_pts(sx + 9.5, bc, 5.0 + grow, 12)
    near_pts = [q for q in body if (q[1] - sy) * sg < 0]
    ring = circ_pts(*p["P"], HUB_R + RING_CLR + RING_WALL + grow)
    # two convex lobes: servo/bolt lobe + arm from the servo's near end to the ring
    return [hull2d(body + main), hull2d(near_pts + ring)]


def make_carrier(p):
    sx, sy = p["S"]
    o = carrier_outline(p)
    c = poly_z(o[0], Z_CARRIER0, Z_CARRIER1).union(poly_z(o[1], Z_CARRIER0, Z_CARRIER1))
    c = c.cut(box(sx - SERVO_W / 2 - SERVO_FIT, sx + SERVO_W / 2 + SERVO_FIT,
                  p["bc"] - SERVO_L / 2 - SERVO_FIT, p["bc"] + SERVO_L / 2 + SERVO_FIT,
                  Z_CARRIER0 - 1, Z_CARRIER1 + 1))
    for ty in p["tabholes"]:
        c = c.cut(cyl_z(sx, ty, 0.9, Z_CARRIER0 - 1, Z_CARRIER1 + 1))
    c = c.cut(cyl_z(*p["P"], HUB_R + RING_CLR, Z_CARRIER0 - 1, Z_CARRIER1 + 1))
    for bx, by in p["bolts"]:                       # slots along Y
        c = c.cut(slot_z(bx, by - ADJUST, bx, by + ADJUST, M3_CLR / 2,
                         Z_CARRIER0 - 1, Z_CARRIER1 + 1))
    for dy in (-4.0, 4.0):                          # zip-tie slot pair
        c = c.cut(box(sx + 9.5 - ZIP_SLOT[0] / 2, sx + 9.5 + ZIP_SLOT[0] / 2,
                      p["bc"] + dy - ZIP_SLOT[1] / 2, p["bc"] + dy + ZIP_SLOT[1] / 2,
                      Z_CARRIER0 - 1, Z_CARRIER1 + 1))
    return c


def make_frame():
    z0, z1, zb = Z_FRAME0, Z_FRAME1, -JAW_REACH
    roof0, roof1 = TOP_Y, TOP_Y + WALL_T
    flo1, flo0 = BOT_Y, BOT_Y - WALL_T
    f = box(PLATE_X_MIN, FRAME_X_MAX, flo0, roof1, z0, z1)
    f = f.union(box(FRAME_X_MIN, PLATE_X_MIN + 1, PICO_CENTER[1] - 16, roof1, z0, z1))
    f = f.union(box(FRAME_X_MIN, FRAME_X_MAX, roof0, roof1, zb, z1))        # roof
    f = f.union(box(PLATE_X_MIN, FRAME_X_MAX, flo0, flo1, zb, z1))          # floor
    f = f.union(box(FRAME_X_MIN, FRAME_X_MIN + RIB_T, PICO_CENTER[1] - 16, roof1,
                    zb, z1))                                                # end wall
    relief_top = cyl_x(PANEL_H, 0, BEND_R + 1, -50, 500)
    relief_bot = cyl_x(0, 0, BEND_R + 1, -50, 500)
    for x in TOP_RIBS:              # top C-jaws: rest on chassis top, lip on face
        r = box(x - RIB_T / 2, x + RIB_T / 2, PANEL_H, roof0 + 1, zb, 0.01)
        r = r.union(box(x - RIB_T / 2, x + RIB_T / 2, PANEL_H - LIP_LEN, roof0 + 1, 0, z1))
        f = f.union(r.cut(relief_top))
    for x, t in [(x, RIB_T) for x in BOTTOM_RIBS] + [(x, CLAMP_RIB_T) for x in CLAMP_RIBS]:
        r = box(x - t / 2, x + t / 2, flo1 - 1, -JAW_GAP, zb, 0.01)
        r = r.union(box(x - t / 2, x + t / 2, flo1 - 1, LIP_LEN, 0, z1))
        f = f.union(r.cut(relief_bot))
    for x in CLAMP_RIBS:            # thumbscrew holes + insert in floor underside
        zc = -JAW_REACH / 2 - 1
        f = f.cut(cyl_y(x, zc, M3_CLR / 2, flo0 - 1, 1))
        f = f.cut(cyl_y(x, zc, INSERT_D / 2, flo0 - 1, flo0 + 6))
    # X-stop: insert through the end wall, screw tip bears on the chassis side
    f = f.cut(cyl_x(PANEL_H - 7, -JAW_REACH / 2, INSERT_D / 2, FRAME_X_MIN - 1,
                    FRAME_X_MIN + RIB_T + 1))
    # pair openings, carrier bolt slots (along X) + sliding nut grooves
    for p in PAIRS:
        sx, sy = p["S"]
        ro = max(HUB_R + RING_CLR, SERVO_BOSS_R) + ADJUST + 0.5
        f = f.cut(slot_z(sx, sy, *p["P"], ro, z0 - 1, z1 + 1))
        for ty in p["tabholes"]:
            f = f.cut(cyl_z(sx, ty, 1.2 + ADJUST + 0.5, z0 - 1, z1 + 1))
        for bx, by in p["bolts"]:
            f = f.cut(slot_z(bx - ADJUST, by, bx + ADJUST, by, M3_CLR / 2, z0 - 1, z1 + 1))
            g = M3_NUT_AF / math.sqrt(3)
            f = f.cut(box(bx - ADJUST - g, bx + ADJUST + g, by - M3_NUT_AF / 2,
                          by + M3_NUT_AF / 2, z0 - 1, z0 + M3_NUT_T))
    for (x, y, _) in (TOGGLE_HMG, TOGGLE_BRIGHT):
        f = f.cut(cyl_z(x, y, TOGGLE_WINDOW_R, z0 - 1, z1 + 1))
    for dx in (-23.5, 23.5):
        for dy in (-5.7, 5.7):
            f = f.cut(cyl_z(PICO_CENTER[0] + dx, PICO_CENTER[1] + dy,
                            PICO_HOLE_D / 2, z0 - 1, z1 + 1))
    return f


def make_chassis_clamp(x):
    """L-hook on top of the bottom jaw at CLAMP_RIB x (assembly coordinates).

    Base plate lies on the jaw top (y = -JAW_GAP) from the jaw's back end to
    just behind the front wall; the lip rises behind the wall. Screw slot runs
    in Z over the jaw's thumbscrew hole so the hook can be pushed into contact.
    """
    y0 = -JAW_GAP
    z_lip = -CHASSIS_T - CLAMP_GRIP_CLR              # lip face touching the wall
    c = box(x - CLAMP_W / 2, x + CLAMP_W / 2, y0, y0 + CLAMP_BASE_T, -JAW_REACH, z_lip)
    c = c.union(box(x - CLAMP_W / 2, x + CLAMP_W / 2, y0, CLAMP_LIP_H,
                    z_lip - CLAMP_LIP_T, z_lip))
    zc = -JAW_REACH / 2 - 1                          # same as the frame's hole
    s = cq.Workplane("XZ").slot2D(2 * CLAMP_SLOT + M3_CLR, M3_CLR, angle=90) \
        .extrude(-(CLAMP_BASE_T + 2)).translate((x, y0 - 1, zc))
    return c.cut(s)


def clamp_to_print(wp):
    """Lay the L on its side (X face down): the L profile prints as a solid
    outline with no overhangs, layers run along the lip for strength."""
    s = wp.val().rotate(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90)
    bb = s.BoundingBox()
    return s.translate(cq.Vector(-bb.xmin, -bb.ymin, -bb.zmin))


def split_frame(f):
    zb, z1 = -JAW_REACH - 1, Z_FRAME1 + 1
    r0, r1 = TOP_Y, TOP_Y + WALL_T
    b0, b1 = BOT_Y - WALL_T, BOT_Y
    rm, bm = (r0 + r1) / 2, (b0 + b1) / 2
    inner = box(SPLIT_X, LAP[1], r0, rm, zb, z1).union(box(SPLIT_X, LAP[1], bm, b1, zb, z1))
    outer = box(LAP[0], SPLIT_X, rm, r1, zb, z1).union(box(LAP[0], SPLIT_X, b0, bm, zb, z1))
    inner_i = f.intersect(box(SPLIT_X, LAP[1], r0, rm, zb, z1)
                          .union(box(SPLIT_X, LAP[1], bm, b1, zb, z1)))
    outer_i = f.intersect(box(LAP[0], SPLIT_X, rm, r1, zb, z1)
                          .union(box(LAP[0], SPLIT_X, b0, bm, zb, z1)))
    left = f.intersect(box(-100, SPLIT_X, -200, 200, zb, z1)).cut(outer).union(inner_i)
    right = f.intersect(box(SPLIT_X, 600, -200, 200, zb, z1)).cut(inner).union(outer_i)
    for x, z in LAP_BOLTS:
        h = cyl_y(x, z, M3_CLR / 2, b0 - 1, r1 + 1)
        left, right = left.cut(h), right.cut(h)
    return left, right


# =============================================================================
# Placeholders (assembly only)
# =============================================================================
def make_servo(p):
    sx, sy = p["S"]
    zb = Z_SERVO_BASE
    s = box(sx - SERVO_W / 2, sx + SERVO_W / 2, p["bc"] - SERVO_L / 2,
            p["bc"] + SERVO_L / 2, zb - SERVO_H, zb)
    s = s.union(box(sx - SERVO_W / 2, sx + SERVO_W / 2, p["bc"] - SERVO_TAB_L / 2,
                    p["bc"] + SERVO_TAB_L / 2, zb - SERVO_TAB_Z - SERVO_TAB_T,
                    zb - SERVO_TAB_Z))
    s = s.union(cyl_z(sx, sy, SERVO_BOSS_R, zb - SERVO_H - SERVO_BOSS_H, zb - SERVO_H))
    return s


def make_shaft(p):
    return cyl_z(p["S"][0], p["S"][1], SPLINE_OD / 2 - SPLINE_TOOTH_H,
                 Z_SPLINE_TIP, Z_CASE_TOP)


def make_panel_bits(post_rot):
    # Open-bottom U channel of CHASSIS_T sheet (bottom is open: the clamps' lips
    # hook up behind the front wall's bare bottom edge).
    bits = {"chassis": box(0, PANEL_W, 0, PANEL_H, -CHASSIS_D, 0)
            .edges("|X and >Z and >Y").fillet(BEND_R)
            .cut(box(CHASSIS_T, PANEL_W - CHASSIS_T, -1, PANEL_H - CHASSIS_T,
                     -CHASSIS_D + CHASSIS_T, -CHASSIS_T))}
    for p in PAIRS:
        x, y = p["P"]
        n = cq.Workplane("XY").polygon(6, NUT_AF / math.cos(math.pi / 6)).extrude(NUT_PROUD)
        bits[f"nut_{p['i']}"] = n.translate((x, y, 0))
        bits[f"bushing_{p['i']}"] = cyl_z(x, y, BUSHING_D / 2, NUT_PROUD, BUSHING_PROUD)
        bits[f"post_{p['i']}"] = d_profile(POST_D / 2, POST_FLAT - POST_D / 2,
                                           BUSHING_PROUD, POST_TIP) \
            .rotate((0, 0, 0), (0, 0, 1), post_rot[p["i"]]).translate((x, y, 0))
    for k, (x, y, _) in {"tog_hmg": TOGGLE_HMG, "tog_bright": TOGGLE_BRIGHT,
                         "tog_power": POWER_TOGGLE}.items():
        bits[k] = cyl_z(x, y, 3.0, 0, 16)
    bits["jack"] = cyl_z(INPUT_JACK[0], INPUT_JACK[1], 6.0, 0, 8)
    bits["led"] = cyl_z(POWER_LED[0], POWER_LED[1], 2.5, 0, 3)
    return bits


# =============================================================================
# Build, check, export
# =============================================================================
def vol_overlap(a, b):
    sa, sb = (a.val() if hasattr(a, "val") else a), (b.val() if hasattr(b, "val") else b)
    ba, bb = sa.BoundingBox(), sb.BoundingBox()
    if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or
            bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
        return 0.0
    try:
        return sa.intersect(sb).Volume()
    except Exception:
        return -1.0


def main():
    os.makedirs(OUT, exist_ok=True)
    fails = []
    ok = lambda c, msg: print(("  PASS " if c else "  FAIL ") + msg) or (c or fails.append(msg))

    print("== Gear geometry")
    cd = MODULE * (Z_SERVO + Z_POT) / 2
    ok(abs(cd - CENTER_DIST) < 1e-6,
       f"center distance m*(z1+z2)/2 = {cd:.2f} vs CENTER_DIST {CENTER_DIST:.2f}")
    print(f"  ratio {Z_SERVO / Z_POT:.2f} -> 180 deg servo gives "
          f"{180 * Z_SERVO / Z_POT:.0f} deg at the pot")
    ok(MODULE >= 1.0, f"module {MODULE} >= 1.0 for FDM")
    zs_abs = Z_GEAR1 + SETSCREW_ABOVE_FACE
    ok(POST_TIP >= zs_abs + 1.0, f"set screw (z={zs_abs:.1f}) bites the post "
       f"(post tip z={POST_TIP:.1f}); D engagement {min(POST_TIP, HUB_TOP) - Z_GEAR0:.1f} mm")
    web = Z_SG_BORE_END - Z_SG_CBORE
    eng = min(SPLINE_LEN, Z_SG_HUB_END - Z_SPLINE_TIP)
    ok(eng >= 0.9 * SPLINE_LEN, f"servo spline engagement {eng:.1f} mm of {SPLINE_LEN:.1f} "
       f"(hub end z={Z_SG_HUB_END:.1f}, case top z={Z_CASE_TOP:.1f})")
    ok(web >= 1.2 and Z_SG_CBORE - Z_GEAR0 <= SCREW_HEAD_H + 0.31,
       f"center screw head {Z_SG_CBORE - Z_GEAR0 - SCREW_HEAD_H:.1f} mm below face, web {web:.1f} mm, "
       f"M2x{SCREW_LEN:.0f} reaches {SCREW_LEN - web - SPLINE_TIP_CLR:.1f} mm into the shaft")
    print(f"  z stack: gears {Z_GEAR0:.1f}-{Z_GEAR1:.1f}, frame {Z_FRAME0:.1f}-{Z_FRAME1:.1f},"
          f" carrier {Z_CARRIER0:.1f}-{Z_CARRIER1:.1f}, servo base {Z_SERVO_BASE:.1f}")

    print("== Building parts")
    sg_local, pg_local = make_servo_gear(), make_pot_gear()
    frame = make_frame()
    fl, fr = split_frame(frame)
    rig = {"frame_left": fl, "frame_right": fr}
    hw = {}
    carriers_unique = {}
    post_rot = {}
    for p in PAIRS:
        i, (sx, sy), (px, py) = p["i"], p["S"], p["P"]
        d = math.degrees(math.atan2(py - sy, px - sx))
        rig[f"servo_gear_{i}"] = sg_local.rotate((0, 0, 0), (0, 0, 1), d) \
            .translate((sx, sy, Z_GEAR0))
        post_rot[i] = d + 180 + 180 / Z_POT
        rig[f"pot_gear_{i}"] = pg_local.rotate((0, 0, 0), (0, 0, 1), post_rot[i]) \
            .translate((px, py, Z_GEAR0))
        c = make_carrier(p)
        rig[f"carrier_{i}"] = c
        key = f"carrier_{'top' if p['sgn'] > 0 else 'bottom'}_{p['ang']:+.0f}deg"
        carriers_unique.setdefault(key, [c, 0])[1] += 1
        hw[f"servo_{i}"] = make_servo(p)
        hw[f"shaft_{i}"] = make_shaft(p)
    pico = box(PICO_CENTER[0] - 25.5, PICO_CENTER[0] + 25.5, PICO_CENTER[1] - 10.5,
               PICO_CENTER[1] + 10.5, Z_FRAME1, Z_FRAME1 + 8)
    hw["pico2"] = pico
    for x in CLAMP_RIBS:
        rig[f"chassis_clamp_{x:.0f}"] = make_chassis_clamp(x)
    panel = make_panel_bits(post_rot)

    print("== Bed fit (print orientation)")
    prints = {"frame_left": (to_print(fl, True), 1), "frame_right": (to_print(fr, True), 1),
              "servo_gear": (to_print(sg_local), 7), "pot_gear": (to_print(pg_local), 7)}
    for k, (c, n) in carriers_unique.items():
        prints[k] = (to_print(c), n)
    prints["chassis_clamp"] = (clamp_to_print(rig[f"chassis_clamp_{CLAMP_RIBS[0]:.0f}"]),
                               len(CLAMP_RIBS))
    for name, (s, n) in prints.items():
        bb = s.BoundingBox()
        ok(bb.xlen <= BED[0] and bb.ylen <= BED[1] and bb.zlen <= BED[2],
           f"{name} x{n}: {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm, "
           f"{len(s.Solids())} solid(s)")
        cq.exporters.export(cq.Workplane().add(s), os.path.join(OUT, f"{name}.step"))

    print("== Collisions (all part pairs, incl. meshing gears and placeholders)")
    bodies = {**rig, **hw, **panel}
    worst = []
    for (na, a), (nb, b) in combinations(bodies.items(), 2):
        if na.startswith(("nut", "bushing", "post", "chassis", "tog", "jack", "led")) and \
                nb.startswith(("nut", "bushing", "post", "chassis", "tog", "jack", "led")):
            continue
        v = vol_overlap(a, b)
        if v > 0.05 or v < 0:
            worst.append((na, nb, v))
    ok(not worst, "no overlapping bodies" + "".join(
        f"\n         {a} x {b}: {v:.2f} mm^3" for a, b, v in worst))

    print("== Chassis clamps (hook behind the 1 mm front wall, locked by the jaw screw)")
    for x in CLAMP_RIBS:
        cl = rig[f"chassis_clamp_{x:.0f}"]
        hits = [k for k, s in {**rig, **hw, "chassis": panel["chassis"]}.items()
                if k != f"chassis_clamp_{x:.0f}" and
                k.startswith(("frame", "carrier", "servo", "shaft", "chassis"))
                and vol_overlap(cl, s) > 0.05]
        ok(not hits, f"clamp at x={x:.0f} clears frame, chassis, carriers, servos and gears"
           + (f": hits {hits}" if hits else ""))
    screw = CLAMP_BASE_T + (-JAW_GAP - (BOT_Y - WALL_T))      # head seat to floor underside
    print(f"  screw path: head on clamp at y={-JAW_GAP + CLAMP_BASE_T:.1f}, insert at "
          f"y={BOT_Y - WALL_T + 6:.1f}..{BOT_Y - WALL_T:.1f} -> M3 x {math.ceil(screw / 5) * 5:.0f} "
          f"fully engages the frame insert ({screw:.1f} mm to the floor underside)")

    print(f"== Adjustment travel (+/-{ADJUST} mm per pair)")
    grown = {}
    for p in PAIRS:
        o = carrier_outline(p, ADJUST)
        grown[f"carrier_{p['i']}_travel"] = poly_z(o[0], Z_CARRIER0, Z_CARRIER1 + 20)\
            .union(poly_z(o[1], Z_CARRIER0, Z_CARRIER1 + 20))
    bad = [(a, b) for (a, sa), (b, sb) in combinations(grown.items(), 2) if vol_overlap(sa, sb) > 0.01]
    bad += [(a, "pico2") for a, s in grown.items() if vol_overlap(s, pico) > 0.01]
    ok(not bad, "carriers (+servos) stay clear of each other and the Pico at full travel"
       + ("" if not bad else f": {bad}"))
    print(f"  frame openings: boss/hub circles keep 0.5 mm margin at full travel (by construction)")

    print("== Reachable controls (keep-out cylinders vs rig, incl. carrier travel)")
    for nm, (x, y, r) in {"HMG toggle": TOGGLE_HMG, "Bright toggle": TOGGLE_BRIGHT,
                          "power toggle": POWER_TOGGLE, "power LED": POWER_LED,
                          "input jack": INPUT_JACK}.items():
        ko = cyl_z(x, y, r, 0, KEEPOUT_DEPTH)
        hits = [k for k, s in {**rig, **hw, **grown}.items() if vol_overlap(ko, s) > 0.01]
        ok(not hits, f"{nm} at ({x:.0f},{y:.0f}) r{r:.0f} clear" + (f": hits {hits}" if hits else ""))

    print("== Writing assembly.step")
    asm = cq.Assembly()
    colors = {"frame": "gray", "carrier": "orange", "servo_gear": "green", "chassis_clamp": "red",
              "pot_gear": "yellow", "servo": "blue", "shaft": "white", "pico2": "darkgreen"}
    for k, s in {**rig, **hw, **panel}.items():
        col = next((v for c, v in colors.items() if k.startswith(c)), "black")
        if k == "chassis":
            col = "lightgray"
        asm.add(s, name=k, color=cq.Color(col))
    asm.save(os.path.join(OUT, "assembly.step"))

    print(f"\nPrint list: " + ", ".join(f"{k} x{n}" for k, (_, n) in prints.items())
          + f"  (total {sum(n for _, n in prints.values())} prints)")
    print("RESULT:", "ALL CHECKS PASSED" if not fails else f"{len(fails)} CHECK(S) FAILED")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
