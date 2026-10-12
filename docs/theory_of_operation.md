# Theory of operation

## Mechanism

Three arms at 120 deg. Each arm is a **lever** pivoting on the rear bulkhead. A bronze
**gear sector** keyed to the lever's pivot shaft meshes a **single-start worm** (lead angle
3.6 deg, friction 0.10: self-locking with margin 1.7). The three worms are driven through
bevel pinions by **one central crown gear**, turned by the expansion stepper through a
planetary gearhead. Consequences:

- all three levers always sit at the same angle (synchronous, self-centring);
- the wall cannot back-drive the worms, so the arms **hold with the power off**;
- motor to lever ratio is 338 / 600 / 990 : 1 (S / M / L), so a microstep is a fraction of
  a micrometre at the wheel.

At the lever tip a **rocker** (bogie) carries two wheels and pivots freely, so both wheels
load equally and the rocker follows dents and bends. The rocker rides on a **sprung slide**
(+/-5 mm): it absorbs weld beads, steps and ovality, and a hall sensor on the slide
measures the wall force (force = preload + k x slide). At the working grip the slide sits
mid-travel.

Each rocker has a **drive pod**: stepper -> self-locking worm gearbox -> serpentine belt to
both wheels. The drive worm also self-locks, so the robot does not roll down a vertical
pipe when the motors stop.

The camera pod is joined to the body by a sprung, booted **U-joint** so the two rigid
segments are short enough for a 3D bend.

**Why not a cone and leadscrew?** The radial stroke is about twice the body radius on every
size. A wedge needs ramp length plus travel (2 x stroke / mechanical advantage) of free axial
space beside the follower, which the body does not have (`rej_*` values in the calcs).

## Grip

Required total normal force in a vertical wet pipe: N = SF x m g / mu_wet with SF = 2,
mu_wet = 0.42. Split over three arms this is the **working grip** (S 28, M 65, L 234 N).
The grip sensor saturates when a rocker reaches its stop, so the force servo never targets
more than `GRIP_LIMIT_N` (sensor max minus overshoot). In a horizontal pipe the bottom arm
also carries 2/3 of the weight; the structure is sized for that.

## Control (RP2040, firmware/)

- **Drive loop 1 kHz**: per-pod speed from 10 ms encoder windows, PID on the TMC velocity,
  slip detection (commanded vs measured vs ToF/IMU), bend assist (differential speeds).
- **Expansion loop 200 Hz**: modes LOCKED / DIAMETER / FORCE / RETRACT / MAINTAIN. Lever angle
  = stepper dead-reckoning blended with the absolute encoder; squeeze is judged on the
  median arm; reported pipe ID = lever reach minus slide and tread squash.
- **State machine**: IDLE, EXPANDING, GRIPPED, CRAWLING, HOLDING, RETRACTING, FAULT,
  LOW_POWER_SAFE. Motion needs a grip; faults stop motion but never release the grip.
- **Safety**: every motion command has a TTL (drive 200-300 ms); heartbeat loss stops;
  hardware E-stop loop cuts the driver enables; watchdog; thermal derate at 70 C, trip 85 C.

## Power and data

48 V over the tether (SELV), bucks to 12 V (24 V on L) and 5 V. M and L carry a Li-ion pack
behind an ideal-diode OR so a tether fault switches to battery without a reset. Data:
100BASE-TX over two shielded pairs in the tether to the reel, which hosts the Wi-Fi access
point. **No radio is used inside the pipe** (steel and wet soil block it).

## Software stack

`MCU <-USB/UART framed protocol-> Pi (FastAPI, WebRTC video, recorder) <-Ethernet tether->
reel Wi-Fi AP <-> phone/tablet dashboard (PWA)`. Frames: `AA 55 | len u16 | type u8 |
payload | CRC16-CCITT`, generated from `protocol/pipebot_protocol.yaml` for C++, Python and
TypeScript.

## Design history

See the revision notes at the top of `params/bot_params.yaml` (R2.x from the calcs, R3.x
from CAD packaging and simulation) and `docs/real_world_issues.md` Part 1.
