# HDRX20 knob servo rig

Seven SG92R servos turn the seven pot posts of the PRS HDRX20 chassis through
1.8:1 step-up gears (36T on the servo, 20T on the pot, module 1, 28 mm centers).
A 180° servo gives about 324° at the pot.

`python servo_rig.py` (conda env `sci`, needs `cadquery`) rebuilds everything,
runs the checks, and writes `step/*.step`. Parts are exported in print
orientation. `step/assembly.step` shows the rig on placeholder panel, posts,
servos and horns.

## How it works
- **Pair** = servo + carrier + servo gear + pot gear. The carrier holds the servo,
  and its bearing ring rides on the pot gear hub, so the center distance comes
  from the carrier and not from the frame.
- Each carrier bolts to the frame through **crossed slots**: carrier slots run in Y
  and frame slots run in X, with sliding nut grooves on the back of the frame. Each
  pair can be trimmed **±3 mm in X and Y** at assembly, so post position errors are
  fixed by adjusting instead of reprinting.
- **Frame**: a plate with a roof and floor, made as two halves joined by 4 + 4 mm lap joints
  in the roof and floor (4 × M3). C-jaws clamp the chassis just behind the face
  (≤22 mm back, clear of the tubes). The top jaws rest on the chassis top and the lips
  sit on the face, with a relief for the rounded bend. Three bottom jaws have
  thumbscrews that push up on the chassis bottom. A screw in the left end wall sets
  X against the chassis side. Finger windows sit over the HMG and Bright toggles. The
  power switch, LED and input jack lie outside the frame.
- Servo gears alternate below and above the post row. The top ones are swung 30° left
  (`SERVO_ANGLE`) to keep the toggles clear.
- **The frame reaches about 32 mm below the chassis bottom**, so stand the amp on
  ≥40 mm risers or let the front overhang the table edge by about 30 mm.

## Print list (PETG, 0.2 mm layers, 4 walls; 23 prints, no supports)
| STEP file | Qty | Orientation |
|---|---|---|
| `frame_left.step` | 1 | as exported (user face down, ribs up) |
| `frame_right.step` | 1 | as exported |
| `carrier_bottom_-90deg.step` | 4 | flat |
| `carrier_top_+120deg.step` | 3 | flat |
| `servo_gear.step` | 7 | flat, horn pocket up |
| `pot_gear.step` | 7 | gear face down, hub up |

Largest part is 215 × 109 × 38 mm. Gears and carriers can share plates.

## Hardware
- **M3 (29 screws, 26 nuts, 4 heat-set inserts, 22 washers):**
  - 14 × M3x8 + washer + nut: carriers to frame (nuts slide in the frame grooves)
  - 7 × M3x6–8 grub screw + nut: pot gear set screws (nut trap in the hub)
  - 3 × M3x35 thumbscrew/SHCS + insert (in the floor underside): bottom clamps
  - 1 × M3x16 + insert + jam nut: X stop
  - 4 × M3x12 + 2 washers + nut: frame lap joint
- **M2:** 14 × M2x8 + nut (servo gear to horn; drill the horn arm holes to 2.0 mm),
  plus 4 × M2 screw, nut and 5 mm standoff for the Pico 2.
- Servo kit: the stock double-arm horn, horn screw and 2 tab self-tappers per servo.
- Zip ties (≤4 mm). Each carrier has a slot pair beside its servo.

## Assembly
1. Press the 4 inserts in (3 in the floor underside, 1 in the left end wall).
2. Bolt the frame halves together at the roof and floor laps.
3. Bolt each servo gear to its horn (M2, heads in the counterbores, nuts on the servo
   side). Center each servo electrically at 90°, then fit the horn and gear and drive the horn
   screw through the center hole.
4. Fit each servo into its carrier from the front (tabs on the front face) and fix it with the
   tab screws. Loosely bolt the carriers to the frame (M3x8, washer, nut in the groove).
5. On the amp, set each pot to mid-travel. Slide a pot gear onto each post until it
   touches the bushing, back it off about 0.5 mm (use a paper shim), and tighten the grub screw on the flat.
6. Offer the frame up: the top jaws go over the chassis top and the carrier rings slide over the
   pot gear hubs while the gears mesh. Push the frame in and left against the X-stop screw,
   then snug the 3 bottom thumbscrews.
7. Each carrier now sits where its pot puts it. Tighten the carrier bolts, then
   run every servo end to end to confirm it moves freely. Zip-tie the cables and mount the Pico 2 on the
   left extension.
8. To refit later, loosen the carrier bolts a little before sliding the frame on. The rings
   relocate each pair. Then retighten.

## Measure before the final print (all in the CONFIG block)
- **`SERVO_BASE_TO_HORN`** (servo base to the outer face of the fitted horn) sets the whole
  depth stack. Also check `SERVO_TAB_Z`, `SERVO_H`, the SG92R body and tab dimensions,
  and `SERVO_SHAFT_FROM_END`.
- Stock horn: `HORN_LEN`, `HORN_HUB_D`, the arm widths, `HORN_T` and `HORN_BOLT_R` (a hole radius you
  will drill out). Set `HORN_TYPE` to match the horn you use.
- `BUSHING_PROUD` (pot gear stop, est. 4.0), `NUT_AF` and `NUT_PROUD` (est. 12.7 / 2.5).
- Control positions: `TOGGLE_HMG`, `TOGGLE_BRIGHT` (est. 25 mm above the posts, midway between
  their pots), `POWER_TOGGLE`, `POWER_LED`, `INPUT_JACK` (est. x=365).
- `BEND_R` (radius of the face-to-top and face-to-bottom bends). Also confirm the chassis sides are
  flush with the face edges (the X stop bears on the left side).
- Measured already: panel size, post x0 ≈ 90, pitch 38.1, post y ≈ 25, D-shaft 6/4.0, post
  length 9. The design works with 9 mm posts: 7 mm D engagement, set screw at z 10.3 against a post tip at z 11.5.
  The ±3 mm carrier travel covers residual post errors.

## Controller (Pico or Pico 2, MicroPython, USB serial)

**Wiring**
- Servo signals go to GP0–GP6 (Pico pins 1, 2, 4, 5, 6, 7, 9), in panel order: Presence, Bass, Middle, Treble, Master Volume, Treble Volume, Bass Volume.
- Servo red goes to a separate 5 V supply rated 4 A or more. Never power the servos from the Pico's VBUS. Put a 470–1000 µF capacitor across that supply.
- Servo brown goes to the supply ground. Tie that ground to a Pico GND pin, e.g. pin 3.

**Flashing**
1. Hold BOOTSEL while plugging in the Pico. Drag over the MicroPython `.uf2` from micropython.org: RP2040 for a Pico, RP2350 for a Pico 2.
2. Copy the firmware over:
   `mpremote cp firmware/main.py :main.py && mpremote reset`

**Use** (from `firmware/`):
- Check the connection with `python rig.py ping`.
- Calibrate each knob with `python rig.py raw <Knob> <us>`, then record where the pointer lands with `python rig.py cal <Knob> <us> <dial>`. Record at least 0, 5 and 10.
- After that, `python rig.py set Presence=5 Bass=7.5` moves knobs by dial value, and `python rig.py off` stops the pulses.

The firmware always approaches a target from below, so gear slack lands the same way every time. Turn pulses off before recording: idle servos can buzz and inject noise into the audio.
