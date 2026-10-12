# Real-world issues: cross-check and FMEA

This file answers one question per feature: **what goes wrong in a real pipe, and what in
this design deals with it?** Part 1 lists the problems that were *found* by the checks in
this repository (design calcs, CAD interference, software-in-the-loop simulation) and how
each was fixed. Part 2 is the failure-mode table for field use. Part 3 is what the
simulator cannot tell you, and must be proven on hardware.

Every number below comes from `params/bot_params.yaml` through `calc/design_calcs.py`,
`cad/build.py` or `tests/acceptance/run_acceptance.py`; re-run them after changing anything.

---

## Part 1 - Problems found during design and simulation (and their fixes)

| # | Found by | Problem | Fix (where) |
|---|---|---|---|
| 1 | calcs | Cone + leadscrew expansion: radial stroke is ~2x the body radius, so the wedge needs more axial space than the body has. Unpackageable. | Worm-sector levers: 3 self-locking worms driven by one bevel crown (R3.1, `calc/mechanism.py`). |
| 2 | calcs | Drive wheels held by stepper detent + belt only held 5-15 % of the T1 load. Robot slides down a vertical pipe when power drops. | Self-locking worm in every drive pod (R2.3). Worm lock margin checked per size. |
| 3 | calcs | Leadscrew lead angle was computed on the nominal diameter (3.41 deg) - wrong. Pitch diameter gives 3.77 deg, less lock margin. | All self-locking checks use pitch diameter (R2.2). |
| 4 | CAD | Size S could not fit 75 mm: three pods + levers collide inside r ~15 mm. | S re-rated to 102-200 mm (R3.3, R3.13). Below that: push-rod camera. |
| 5 | CAD | One-piece body too long for a 3D elbow on M. | Camera pod on a sprung, booted U-joint (R3.6). T8 checks every size. |
| 6 | CAD | Masses grew past targets. | Pocketing (hollow shafts, sector webs, housing pockets), thin-section bearings, +12 % allowance kept honest (R3.5, R3.8). |
| 7 | SIL | Robot fell in a vertical pipe at boot when the arms started retracted. | Boot assumes "already gripped" if all arms read load; never auto-retracts (firmware `Robot::begin`). |
| 8 | SIL | Spring bottomed at 31 N: preload was set wrong. | Preload = working grip - k x travel, so the slide sits mid-travel at the working grip (gen_params). |
| 9 | SIL | Over-current false trips while crawling. | Limits are a fraction of the motor buck rating, not of a motor (control block in YAML). |
| 10 | SIL | Noisy wheel-speed estimate from encoder differences tripped slip detection. | 10 ms window speed estimate in the drive loop. |
| 11 | SIL | Horizontal pipe: body sag unloads the top arm, which then read 0 N and triggered "no contact". | Gripped = 2 of 3 arms loaded; unloaded top arm counted as preload (`ExpansionLoop::effForce`). |
| 12 | SIL | **Grip sensor saturation.** When a rocker reaches its stop, the hall slide sensor tops out (M: 73.4 N) while the true wall force keeps rising on the frame's stiffness (187 N seen). The force servo kept squeezing - a PVC pipe would crack. | Firmware only servos below `GRIP_LIMIT_N` = sensor max - overshoot band; hold margins use that limit (R3.10). L spring 3.0 -> 3.2 N/mm to keep T1 margin. |
| 13 | SIL | **Horizontal sag load.** With one arm straight down, statics give F_bottom = F0 + 2W/3 (M: 100 N vs 76 N rating). | Lever, sector and worm sized for it: sector faces widened, L lever 5 x 40 mm (R3.11). |
| 14 | SIL | The sag load (item 13) looked like over-squeeze, so the firmware backed the arms off and lost grip. | Squeeze is judged by the **median** arm: squeeze loads all three, sag only one (`gripSqueeze`). |
| 15 | SIL | **Reported pipe diameter 10 mm too large** while gripping: the estimate used lever angle only, ignoring the compressed slides (5 mm each) and tread squash. | Diameter = lever reach - 2 x mean(slide + F/k_contact); mean over 3 arms cancels sag. Now within 0.3 mm (T5). |
| 16 | SIL | **No grip at the end of the rated range:** the stroke was sized for a wheel *touching* the max pipe, not pressing into it; at the min pipe the retracted wheels had zero insertion clearance. | `stroke()` adds full slide travel at max and insertion clearance at min (R3.13). |
| 17 | SIL | **Encoder resolution:** an AS5600 count is 0.09 deg = 0.75 mm of diameter on the 250 mm L lever, so repeatability was > 1 mm. | Lever angle = stepper dead-reckoning (600:1+ gearing) blended with the absolute encoder; the raw encoder still detects stalls. True repeatability 0.01-0.03 mm (T4). |
| 18 | SIL | **Thermal:** a 60 min vertical climb at 40 C ambient hit the 85 C trip on L. | Thermal model from hull area (YAML `thermal`); firmware derates motor current at 70 C and keeps climbing (T12 passes on all sizes, hottest 70.6 C). On hardware: thermal pads from drivers and motors to the aluminium hull are mandatory. |
| 19 | SIL | Packet loss at a bit-error rate of 1e-6 is ~0.1 %, right at the T11 limit (a 1000-bit frame). | CRC drops every corrupt frame (none reach the operator); tether uses shielded twisted pairs, budget BER <= 1e-7 (T11: 0.013 % loss). |
| 20 | SIL | The simulated motor driver ignored the current the firmware set, so thermal derating had no effect on torque. | Simulator torque now scales with run current (`SimStepper::setCurrent`). |
| 21 | browser | Dashboard on a phone: E-STOP pushed off-screen, status banner hidden under the toolbar, charts magnified sensor noise into fake events. | E-STOP floats above the control deck; banner below the toolbar; charts have a minimum span. |
| 22 | build | PlatformIO fetched Unity from its registry for every firmware build (a test folder exists). | `test_dir` moved; unit tests run natively with CMake and vendored Unity. |

## Part 2 - Field FMEA

Severity (S), occurrence (O), detection (D), 1-10; RPN = S x O x D. Rows sorted by risk.

| Failure mode | Effect | Cause | S | O | D | RPN | Prevention / detection / recovery |
|---|---|---|---|---|---|---|---|
| Grip lost in a vertical pipe | Robot falls, damage, stuck at bottom | Biofilm / grease (mu < 0.2), oil-coated wall | 9 | 4 | 4 | 144 | T6 shows biofilm-wet cannot be climbed: **clean or flush the line first**. Slip detection (wheel vs ToF vs IMU) stops and re-grips; tether is the backup, rated 5x weight and never the primary load path. |
| Pipe wider than the stroke (broken joint, washout) | No contact, robot drops | Missing section, socket, eccentric reducer | 9 | 3 | 3 | 81 | NO_CONTACT fault stops the drive before the arms leave the wall; ToF forward ranging; scout with the camera, crawl at <= 25 % near joints. |
| Wheel tread worn / glazed | Lower mu, slip | Abrasion on concrete / rust | 7 | 5 | 4 | 140 | Replaceable overmoulded tread (BOM); tilt-table mu check before each job (test_plan "Pre-job checks"). |
| Tether snag in a bend | Stuck robot | Tether drag in 3+ bends | 7 | 4 | 3 | 84 | Pay out tether by hand, never let it go slack around a bend (no reel tension sensor in this revision); tether drag is in the simulator; retrieval = retract arms + reel in; strain relief at the robot. |
| Stuck at an obstruction (root intrusion, debris) | Mission abort | Pipe condition | 6 | 5 | 3 | 90 | Front ToF + camera; stopping distance from top speed is v^2/2a = 15 / 24 / 18 mm (S / M / L); reverse is always available (self-locking worms drive both ways). |
| Water ingress | Electronics loss | Seal damage, over-depth | 8 | 3 | 3 | 72 | IP68 design: O-ring glands per `docs/assembly_manual.md`; vacuum decay test before each job; humidity + pressure sensors raise faults; flooded gearboxes (grease-filled, no air to pump). |
| Weld bead / offset joint > 5 mm | Arm stall or lost grip | Field welds, misaligned spigots | 6 | 4 | 4 | 96 | Sprung rocker +/-5 mm (T9 passes 5 mm bead and 5 mm step at 80 % speed); bigger steps: slow down, retract-pass-regrip manually. |
| Ovality / dents | Uneven arm loads | Buried plastic pipe deflection | 5 | 5 | 3 | 75 | +/-5 mm compliance per arm absorbs ~5 % ovality; per-arm force shown in the HUD. |
| Over-squeeze cracking thin pipe | Pipe damage | Force servo error, sensor saturation | 9 | 2 | 3 | 54 | Fixed (Part 1 #12): servo ceiling below sensor saturation; MAINTAIN backs off on high median force. Per-material grip limit can be lowered via SET_PARAM `GRIP_TARGET_N`. |
| Gas in a sewer / explosive atmosphere | Ignition | Methane, H2S | 10 | 2 | 6 | 120 | **This robot is not ATEX/IECEx certified.** The gas sensor is a warning only. Gas-free permit, ventilation, and confined-space rules apply. |
| Camera fogging | No video | Warm robot into cold wet pipe | 4 | 6 | 2 | 48 | Window heater output (GP13) and a hydrophobic window coating; IR mode for murky water. |
| Lost link (tether damage, Ethernet drop) | Robot stops | Abrasion, connector | 5 | 3 | 1 | 15 | Command TTL stops motion in <= 300 ms; heartbeat loss stops and holds grip; worms hold unpowered (T14). |
| Tether cut / power loss mid-climb | Fall | Accident | 9 | 2 | 1 | 18 | T14: stops in 20-240 ms, zero sink, holds unpowered (self-locking drive + expansion). M/L switch to battery. |
| Battery exhausted (M/L) | Stuck | Long job | 5 | 3 | 2 | 30 | LOW_POWER_SAFE holds grip and stops; T15 runtime M 149 min, L > 180 min in the simulator (calc: 68 / 93 min conservative). |
| Encoder failure | Wrong diameter / odometry | Magnet slip, cable | 5 | 3 | 2 | 30 | ENCODER_FAIL fault; lever angle continues by stepper dead-reckoning; odometry falls back to median of arms + ToF. |
| Motor stall (missed steps) | Arms out of sync | Jam, debris in worm | 6 | 2 | 2 | 24 | Raw-encoder vs commanded rate stall detector, EXPANSION_STALL fault; worms are synchronous through one crown so arms never desynchronise. |
| Overheating in hot pipes (steam, sun-baked) | Derate / trip | Ambient > 50 C | 6 | 3 | 2 | 36 | Derate at 70 C, trip at 85 C; do not deploy above 50 C ambient without the thermal test. |

## Part 3 - What the simulator cannot prove (do these on hardware)

- **Friction values** of your tread on your pipes. The `pipe_materials` table in the YAML
  is an estimate. Measure it with the tilt-table test in `docs/test_plan.md`, then re-run
  `make params acceptance`.
- **Video latency** (T10) and **sealing** (T13): bench procedures in `docs/test_plan.md`.
- **3D contact in elbows**: the simulator is 1-D along the pipe. Elbow passage is checked
  geometrically (T8) and in CAD; prove it in a clear acrylic elbow before the field.
- **Spring and hall calibration**: per-robot, at assembly (`docs/test_plan.md` "Calibration").
- **Long-term wear**: worm/bronze wear, tread wear, seal compression set.
- **Electromagnetic noise** of 48 V switching on the Ethernet pairs over 50 m.

## Known deviation

**T5 centring in horizontal pipes**: the body sags about 6 mm onto the bottom arm's
spring (vertical pipes: 0 mm). Getting under 3 mm needs springs stiff enough
(k >= 2W/9: S 3.9, M 9.2, L 33 N/mm) to lose the +/-5 mm compliance that keeps grip over
joints, beads and ovality. Grip safety wins. The camera image and the reported pipe
diameter are unaffected (the diameter is computed from all three arms; T5 shows 0.06-0.3
mm error). If you need a centred camera in horizontal lines, use a stiffer spring set and
accept a smaller step allowance.
