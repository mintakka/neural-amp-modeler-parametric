# HDRX20 knob servo rig

Seven SG92R servos turn the seven pot posts of the PRS HDRX20 chassis through
1.8:1 step-up gears (36T on the servo, 20T on the pot, module 1, 28 mm centers).
A 180° servo gives about 324° at the pot.

`python servo_rig.py` (conda env `sci`, needs `cadquery`) rebuilds everything,
runs the checks, and writes `step/*.step`. Parts are exported in print
orientation. `step/assembly.step` shows the rig on placeholder panel, posts,
servos and output shafts.

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
  sit on the face, with a relief for the rounded bend. The chassis bottom is open
  (bare 1 mm edge, no flange), so the three bottom jaws carry a **chassis clamp**
  each: an L-hook that sits on top of the jaw and reaches up behind the front wall;
  the wall is pinched between the hook lip (inside) and the jaw lip (outside), and the
  jaw's M3 screw locks the hook. A screw in the left end wall sets
  X against the chassis side. Finger windows sit over the HMG and Bright toggles. The
  power switch, LED and input jack lie outside the frame.
- Servo gears alternate below and above the post row. The top ones are swung 30° left
  (`SERVO_ANGLE`) to keep the toggles clear.
- **The frame reaches about 32 mm below the chassis bottom**, so stand the amp on
  ≥40 mm risers or let the front overhang the table edge by about 30 mm.

## Print list (PETG, 0.2 mm layers, 4 walls; 26 prints, no supports)
- `chassis_clamp` × 3 (21 × 11 × 8 mm, prints on its side as exported; ~2 g each)
| STEP file | Qty | Orientation |
|---|---|---|
| `frame_left.step` | 1 | as exported (user face down, ribs up) |
| `frame_right.step` | 1 | as exported |
| `carrier_bottom_-90deg.step` | 4 | flat |
| `carrier_top_+120deg.step` | 3 | flat |
| `servo_gear.step` | 7 | gear face down, spline hub up |
| `pot_gear.step` | 7 | gear face down, hub up |

Largest part is 215 × 109 × 38 mm. Gears and carriers can share plates.

## Hardware
- **M3 (29 screws, 26 nuts, 4 heat-set inserts, 22 washers):**
  - 14 × M3x8 + washer + nut: carriers to frame (nuts slide in the frame grooves)
  - 7 × M3x6–8 grub screw + nut: pot gear set screws (nut trap in the hub)
  - 3 × M3x35 SHCS (or the thumbscrews) + washer + insert (in the floor underside):
    chassis clamps, screwed from inside the open chassis bottom (M3x30 also works, 3.5 mm engagement)
  - 1 × M3x16 + insert + jam nut: X stop
  - 4 × M3x12 + 2 washers + nut: frame lap joint
- **M2:** 4 × M2 screw, nut and 5 mm standoff for the Pico 2.
- Servo kit: the center screw and 2 tab self-tappers per servo (horns not used). The
  stock center screw is short. With an M2x6 or 2 × 6 mm self-tapper it reaches 3.6 mm into
  the shaft, through a 2.1 mm web.

## Servo gear (direct spline drive)
The servo gear presses straight onto the SG92R output spline: a printed internal spline
(`SPLINE_TEETH` 21, `SPLINE_OD` 4.8, `SPLINE_TOOTH_H` 0.3, 3 mm engagement, all ESTIMATED).
The servo's center screw holds it, and its head sits 0.3 mm below the gear face in a shallow
counterbore. The teeth stay in the same plane as before, and the hub stops 0.5 mm short of the servo case.
**Fit tuning:** `SPLINE_FIT` (radial, default 0.05). If the gear won't press on, raise it
by 0.05. If it slips or wobbles, lower it by 0.05 (negative is fine). Print one test gear
first. If your servos measure larger than 4.8 mm over the spline (some clones run 4.9 mm or more), set
`SPLINE_OD` to the caliper value.
- Zip ties (≤4 mm). Each carrier has a slot pair beside its servo.

## Assembly
1. Press the 4 inserts in (3 in the floor underside, 1 in the left end wall).
2. Bolt the frame halves together at the roof and floor laps.
3. Center each servo electrically at 90°. Press its servo gear onto the spline (hub
   first) and drive the center screw through the gear into the shaft.
4. Fit each servo into its carrier from the front (tabs on the front face) and fix it with the
   tab screws. Loosely bolt the carriers to the frame (M3x8, washer, nut in the groove).
5. On the amp, set each pot to mid-travel. Slide a pot gear onto each post until it
   touches the bushing, back it off about 0.5 mm (use a paper shim), and tighten the grub screw on the flat.
6. Offer the frame up: the top jaws go over the chassis top and the carrier rings slide over the
   pot gear hubs while the gears mesh. Push the frame in and left against the X-stop screw.
   Through the open chassis bottom, set a chassis clamp on top of each bottom jaw with its
   lip up behind the front wall, drop an M3x35 (washer under the head) through the clamp's
   slot and the jaw hole, push the clamp forward so the lip bears on the inside of the wall,
   and tighten into the floor insert. No new inserts are needed.
7. Each carrier now sits where its pot puts it. Tighten the carrier bolts, then
   run every servo end to end to confirm it moves freely. Zip-tie the cables and mount the Pico 2 on the
   left extension.
8. To refit later, loosen the carrier bolts a little before sliding the frame on. The rings
   relocate each pair. Then retighten.

## Measure before the final print (all in the CONFIG block)
- `SERVO_BASE_TO_GEAR` is fixed by the printed frame and carriers.
  `SERVO_BASE_TO_SPLINE_TIP` (est. 29.9) and `SERVO_BOSS_H` set the spline engagement and
  the hub-to-case gap. Also check `SERVO_TAB_Z`, `SERVO_H`, the SG92R body and tab dimensions,
  and `SERVO_SHAFT_FROM_END`.
- Spline: `SPLINE_OD`, `SPLINE_TEETH`, `SPLINE_LEN`, then tune `SPLINE_FIT`.
- `BUSHING_PROUD` (pot gear stop, est. 4.0), `NUT_AF` and `NUT_PROUD` (est. 12.7 / 2.5).
- Control positions: `TOGGLE_HMG`, `TOGGLE_BRIGHT` (est. 25 mm above the posts, midway between
  their pots), `POWER_TOGGLE`, `POWER_LED`, `INPUT_JACK` (est. x=365).
- `BEND_R` (radius of the face-to-top and face-to-bottom bends). Also confirm the chassis sides are
  flush with the face edges (the X stop bears on the left side).
- Measured already: panel size, post x0 ≈ 90, pitch 38.1, post y ≈ 25, D-shaft 6/4.0, post
  length 9. The design works with 9 mm posts: 7 mm D engagement, set screw at z 10.3 against a post tip at z 11.5.
  The ±3 mm carrier travel covers residual post errors.

## Controller (Pico or Pico 2, Pico SDK C++, USB serial)

**Wiring**
- Servo signals go to GP0–GP6 (Pico pins 1, 2, 4, 5, 6, 7, 9), in panel order: Presence, Bass, Middle, Treble, Master Volume, Treble Volume, Bass Volume.
- Servo red goes to a separate 5 V supply rated 4 A or more. Never power the servos from the Pico's VBUS. Put a 470–1000 µF capacitor across that supply.
- Servo brown goes to the supply ground. Tie that ground to a Pico GND pin, e.g. pin 3.

**Flashing** (needs the Pico SDK at `~/Projects/pico/pico-sdk`, or set `PICO_SDK_PATH`)
1. Build it from `firmware/`:
   `cmake -B build && cmake --build build`
   For a Pico 2, add `-DPICO_BOARD=pico2` to the first command.
2. Hold BOOTSEL while plugging in the board, then drag `build/servo_rig.uf2` onto it.

**How the servos move** (SG92R datasheet: 1000–2000 µs ≈ 180°, about 5.5 µs per degree):
- **Range:** the firmware refuses any pulse outside 1000–2000 µs, so a servo can never be driven into its internal stops.
- **Scale:** the pot's full 0–10 is only about 920 µs of servo travel, about 92 µs per dial number.
- **Speed:** the servo itself moves at full speed toward every pulse. All slow motion comes from the firmware: each move is one continuous ramp, about 150 µs/s or 1.6 dial numbers per second, updated every 20 ms. It always starts from the servo's known position.
- **Known positions:** positions are saved in the Pico's flash. A servo whose move was interrupted (for example by power loss) boots as "unknown" and won't move until it is re-homed.
- **Moves:** one servo at a time, which stays within USB power.
- **Slack:** every move finishes with a 20 µs approach from the low-pulse side, so gear slack lands the same way every time.
- **Recording:** turn pulses off before recording. Idle servos can buzz and inject noise into the audio.

**Never turn a meshed knob by hand.** The Pico then no longer knows where that servo is. The only recovery is re-homing it.

**Calibrate each knob once** (from `firmware/`). The top-row servos (2, 4, 6) turn their pots the opposite way, so for them the dial goes *down* as the pulse goes up. Calibration records this per knob.
1. **Re-home.** With that servo's gear **unmeshed**, run `python rig.py home <Knob>`. The servo jumps to centre (1500 µs). It's unmeshed, so the jump is harmless.
2. **Mesh at 5.** Turn the pot by hand to 5, then mesh the gear.
3. **Step toward one end.** Use `python rig.py raw <Knob> <us>` in 25–50 µs steps. After each step, record the dial reading with `python rig.py cal <Knob> <us> <dial>`. **Stop at about 0.3. Never drive to the stop.**
4. **Step toward the other end** the same way, stopping at about 9.7.
5. **Set the limits.** Run `python rig.py limits <Knob> <lo> <hi>` with the pulses at about 0.2 and 9.8. The Pico stores them and refuses anything outside them.
6. **Move by dial value.** `python rig.py set <Knob>=<dial>` now works. It refuses dials beyond the calibrated span.

If a step doesn't change the dial, or the servo buzzes, stop and run `python rig.py off`.
