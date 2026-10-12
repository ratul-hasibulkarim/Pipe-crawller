# Test plan

Two layers:

- **Simulation** (`make acceptance`): T1-T15 run against the real firmware and a physics
  plant; results in `docs/acceptance_report.md`. Run after every change.
- **Hardware** (this document): the same tests on a built robot, plus calibration and
  pre-job checks. A hardware result replaces the simulated one in the report.

Unit tests (`make test`): PID math, CRC and protocol round-trip, state-machine transitions,
self-locking and grip-force calculations, kinematics, safety logic, Pi API, dashboard.

## Calibration (once per robot, at assembly)

1. **Hall grip sensors**: for each arm, read the ADC with the slide on its outer stop (0 mm)
   and with a 10 mm gauge block (full slide). Enter `kHallZeroV` and
   `kHallVPerMm = (V10 - V0) / 10` in `firmware/include/pinmap.h`, rebuild.
2. **Spring rate**: hang known masses on one rocker (vertical fixture), plot force vs slide;
   slope must be within 10 % of `rocker.spring_rate_n_mm`, intercept within 10 % of the
   preload. Otherwise change the spring, not the firmware.
3. **Lever encoder zero**: retract fully (`EXPAND RETRACT`); the reported lever angle must be
   0 deg +/- 0.2. Measure the wheel reach with calipers across two arms and compare with
   `wheel_reach_retracted_dia_mm` in `cad/exports/<size>/params.csv`.
4. **Wheel diameter**: drive 1.000 m along a taped straight edge; set
   `WHEEL_DIA_MM` (Settings > Wheel diameter calibration) so odometry reads 1.000 m.
5. **Tilt-table friction** (per tread batch and per pipe material you will meet): put a pipe
   coupon on a tilting plate, place one wheel pair loaded with a known weight, raise until
   it slides. mu = tan(angle). Do it dry and wet. Enter the result in
   `common.pipe_materials` and rerun `make params acceptance`.

## Pre-job checks (every deployment, ~5 min)

- Dashboard **Connect > Pre-flight**: all green (link, telemetry rate, IMU, camera,
  encoders, no latched faults, supply, grip test).
- Vacuum-decay test of the hull through the service port: -300 mbar, < 10 mbar loss in 5 min.
- E-stop: long-press on the phone stops all motion; the hardware loop button at the reel
  cuts the driver enables (motors free, worms hold).
- Tread condition: no glazing or cuts; tilt check on a coupon of the job's pipe if unsure.
- Tether: no jacket damage in the first 5 m; strain relief seated.

## Acceptance tests on hardware

| Test | Setup | Procedure | Pass |
|---|---|---|---|
| T1 hold, power off, 3x mass | Vertical steel pipe of mid-range ID, robot gripped at the controllable max grip | Hang 2x robot mass from the rear eye (total 3x), cut all power, dial indicator on the body | < 0.1 mm movement in 10 min |
| T2 hold at angles | Pipe on a tilt frame: 0, 30, 45, 60, 90 deg | Grip at the default force, powered 1 min, unpowered 1 min | < 0.1 mm at every angle |
| T3 speed | 3 m straight pipe, horizontal and vertical | Command 25/50/100 % up and 50 % down, time 1 m with the camera on tape marks | each within +/-10 %; all 6 wheels turn (check the belts by camera) |
| T4 diameter sweep | Robot in free air on a stand | Min <-> max diameter 3 times, stopwatch; calipers across the wheels at each end | < 20 s each way, ends repeat within +/-1 mm |
| T5 centring | Pipes at min / mid / max ID, horizontal and vertical | Laser on the camera axis to a target at the pipe end, measure offset | vertical < 3 mm; horizontal: record (documented ~6 mm sag) |
| T6 materials | 1 m coupons of steel, PVC, HDPE, cast iron, concrete-lined, 45 deg | Climb 0.5 m dry, then wet | all climb; biofilm is informational |
| T7 wet traction | Vertical pipe, wheels and wall wetted continuously | Hold 5 min, climb 0.5 m | no slip |
| T8 elbow | 90 deg elbow at the rated bend radius (3x ID), clear acrylic if possible | Drive through both ways | passes without lifting an arm or binding the U-joint |
| T9 bead / step | 5 mm weld bead and 5 mm reducer step in a vertical pipe | Climb across at 80 % speed | no stall, no grip fault |
| T10 video latency | 50 m tether on the reel, phone on the reel Wi-Fi | Film a millisecond clock *and* the phone showing the camera view of that clock with a 240 fps camera; latency = difference of the two readings, 20 samples | median < 150 ms |
| T11 telemetry | 50 m tether, 60 min crawl back and forth | Dashboard Run screen shows packet counter; Pi log `link_reconnects` | >= 20 Hz, loss < 0.1 % |
| T12 thermal | 40 C ambient chamber or heated pipe, 60 min vertical crawl | Thermocouples on drivers, motors, Pi, battery | all < 85 C, robot still climbing |
| T13 IP68 | Tank, 1 m | Vacuum-decay first; then 30 min submerged powered; humidity log | no water, humidity rise < 5 %RH |
| T14 failsafe | Vertical pipe, safety line slack | Climbing at full speed, unplug the tether at the reel | stops within 0.5 s, < 1 mm sink |
| T15 endurance (M/L) | Horizontal pipe loop, battery only | Crawl at 50 % speed until LOW_POWER_SAFE | >= 60 min; robot holds grip after |
