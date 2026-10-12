# Recreate and simulate PIPEBOT-X yourself

Three ways to check this design on your own machine, from quickest to deepest:

1. **Run the simulator that already exists** (software-in-the-loop, 5 minutes, free).
2. **Open the CAD in SolidWorks** (or FreeCAD / Fusion 360 / Onshape): inspect, mate,
   animate the mechanism, run FEA on the loaded parts.
3. **Build a physics model** in a robotics simulator (SolidWorks Motion, Simscape
   Multibody, Gazebo, Webots, MuJoCo) for 3-D contact in elbows.

Every number in this guide is per size and comes from `params/bot_params.yaml`. After you
change the YAML, run `make params cad` and the tables below regenerate in
`cad/exports/<size>/params.csv`.

---

## 1. The simulator in this repository (start here)

The simulator compiles the **real firmware** (state machine, control loops, safety logic,
wire protocol) against a physics model of the robot in a pipe: lever kinematics, sprung
rockers with stops, tread and frame stiffness, stick/slip friction per pipe material,
self-locking worms, stepper torque vs speed and current, tether/battery power, and
temperature.

```bash
# Linux / macOS / WSL: needs cmake, a C++17 compiler, Python 3.11+
pip install -r requirements-dev.txt
make sim                                        # build/sil/pipebot_sil_{S,M,L}
make acceptance                                 # T1-T15 for all sizes -> docs/acceptance_report.md
python tools/plot_run.py tests/acceptance/out/M/T9.csv -o t9.png   # look at any run
```

### Write your own scenario

`build/sil/pipebot_sil_M --scenario my.json --out run.csv --summary sum.json --events ev.jsonl`

```json
{
  "duration_s": 60, "log_hz": 20,
  "plant": {
    "sections": [
      {"start_m": 0,   "end_m": 1.3, "id_mm": 236, "incline_deg": 90},
      {"start_m": 1.3, "end_m": 50,  "id_mm": 226, "incline_deg": 90, "wet": true,
       "mu_dry": 0.55, "mu_wet": 0.32}
    ],
    "x0_m": 1.0, "start_gripped": true, "battery_soc": 0.9, "ambient_c": 30
  },
  "operator": [
    {"cmd": "EXPAND", "mode": "FORCE", "target": 66},
    {"cmd": "DRIVE", "speed": 80, "distance_m": 1.0},
    {"cmd": "WAIT", "duration_s": 5},
    {"cmd": "EXPAND", "mode": "RETRACT"}
  ],
  "events": [
    {"t": 20, "type": "POWER_LOSS"},
    {"t": 25, "type": "EXTRA_MASS", "value": 8.4}
  ]
}
```

Operator steps: `EXPAND` (`FORCE` N per arm | `DIAMETER` mm | `RETRACT`), `DRIVE` (`speed`
mm/s, `duration_s` or `distance_m`), `WAIT`, `WAIT_STATE`, `GIMBAL`, `LIGHT`, `ESTOP`,
`CLEAR_FAULT`, `SET_PARAM`. Events: `POWER_LOSS`, `POWER_RESTORE`, `PULL` (N, tether pull),
`EXTRA_MASS` (kg), `WET`, `ESTOP_LOOP`, `PI_SILENT` (s of no commands), `IMU_FAIL`,
`LEVER_ENC_FAIL`, `AMBIENT` (C), `LINK_BER` (bit-error rate). The CSV has both what the
robot *measured* (grip0..2, est_diameter_mm, ...) and the *truth* (f0_true..,
pipe_id_mm, x_m, theta, off_x_mm, ...), so you can see where the robot's own estimate is
wrong.

### Drive it live from the dashboard

```bash
build/sil/pipebot_sil_M --scenario my.json --tcp 7000 &          # simulated robot, real time
cd app && npm ci && npm run build && cd ..
PIPEBOT_LINK=tcp://127.0.0.1:7000 PIPEBOT_CAMERA=synthetic uvicorn pi.app.main:app --port 8000
# open http://localhost:8000 (or your laptop's IP from a phone on the same Wi-Fi)
```

This is exactly the stack that runs on the robot's Pi; only the link (TCP instead of USB)
and the camera (synthetic) differ.

---

## 2. SolidWorks

### 2.1 Get the files

`make cad` (or download the `cad-exports` artifact from CI) produces, per size:

| File | Use |
|---|---|
| `cad/exports/<size>/assembly.step` | whole robot, arms retracted, one body per part, named |
| `cad/exports/<size>/step/<part>.step` | every part on its own (made and purchased) |
| `cad/exports/<size>/params_solidworks.txt` | every dimension and design load as SolidWorks global variables |
| `cad/exports/<size>/params.csv` | the same for Fusion 360 / FreeCAD / Excel |
| `docs/bom.csv`, `docs/mass_report.md`, `docs/assembly_manual.md` | BOM with material, process, mass, cost; assembly order |

Coordinates: millimetres; **+Z is the pipe axis toward the camera nose**; arm A lies in the
+X/Z plane, arms B and C at +120 / +240 deg about Z.

### 2.2 Import

1. **File > Open**, type *STEP AP203/214/242*, select `assembly.step`.
   *Options...*: tick **Import multiple bodies as parts** (gives a real assembly) and
   **Map configuration data**. Units: millimetres.
2. Run **Import Diagnostics** when asked (it heals gaps from the B-rep export; it should
   report nothing to fix).
3. Save as `PIPEBOT_<size>.SLDASM`; let it save the parts into a `parts/` folder.
4. Materials: apply from the BOM (`docs/bom.csv` column *material*): 6061-T6 for levers,
   shafts, hull; CuSn12 (use "Tin Bronze" in the SolidWorks library) for worm sectors;
   PA12/PETG for printed parts. **Tools > Evaluate > Mass Properties** should match
   `docs/mass_report.md` "CAD total" within ~1 %. Purchased parts are simplified envelopes:
   override their mass with the catalogue value.

The imported parts are dumb solids (no feature tree). That is enough for inspection, mates,
motion and FEA. If you want an editable model:

### 2.3 Rebuild a part parametrically (optional)

1. New part. **Tools > Equations > Import...**, choose `params_solidworks.txt`. All
   dimensions (e.g. `"lever_length_mm" = 116mm`, `"lever_plate_height_mm" = 14mm`) become
   global variables. Tick **Link to external file** so re-importing after a YAML change
   updates the model.
2. Sketch over the imported STEP of the same part (insert it as a reference body), set each
   dimension to `= "global_name"`.
3. Key relations you must keep (they are the design, the rest is packaging):
   - wheel reach (diameter) = 2 x (`lever_pivot_radius_mm` + `lever_length_mm` sin(theta)
     + `rocker_axle_offset_mm` + `wheel_dia_mm`/2)
   - theta range = `lever_angle_retracted_deg` .. `lever_angle_extended_deg`
   - worm sector: module `expansion_sector_module_mm`, pitch radius = module x teeth / 2,
     face `expansion_sector_face_mm`; worm pitch dia `expansion_worm_pitch_dia_mm`,
     1 start (lead angle 3.58 deg: self-locking).

### 2.4 Mates that make it move

Make one sub-assembly per arm (lever plates + worm sector + pivot shaft + rocker + slide
bushing + spring retainer + 2 wheels + drive pod), then:

| Joint | Mate |
|---|---|
| body | Fix `body_mid_chassis` |
| lever pivot | Concentric pivot shaft <-> bulkhead bore + Coincident shaft shoulder; lever plates and sector **Lock** to the shaft (they are keyed by the D-flat) |
| lever range | **Angle (limit)** between lever plate and body XZ plane, min/max = `lever_angle_retracted_deg` / `lever_angle_extended_deg` |
| worm -> sector | **Mechanical mates > Gear**, ratio 1 : `expansion_sector_teeth` (single-start worm), select the worm cylinder and the sector pitch face |
| crown -> worm pinions | Gear mate, ratio `expansion_bevel_ratio` |
| motor -> crown | Gear mate, ratio `expansion_planetary_ratio` |
| rocker | Concentric rocker pin <-> lever tip hole (free rotation, the bogie) |
| sprung slide | **Slot** mate or **Distance (limit)** 0 .. 2 x `compliance_travel_mm` (= 10 mm) |
| wheels | Concentric to their axles, free rotation |
| camera pod | Universal joint: two Concentric mates through the spider, Angle limit +/-45 deg |

Check: drag one lever. All three must move together (one crown drives all worms), and
**Tools > Evaluate > Interference Detection** at the retracted, middle and extended angles
should show only the intended contacts (gear meshes, shaft fits). The repository does the
same check in `cad/interference_check.py` at 9 stroke positions; it reports 0 collisions
for S, M and L.

### 2.5 Motion study (SolidWorks Motion: Premium, or Professional + Motion add-in)

Add a pipe: new part, extruded ring with inner diameter = the pipe you want (e.g. 236 mm for
M), length 1500 mm. Insert it, mate its axis to the robot Z axis (for a free simulation,
leave the robot unconstrained and let contact hold it).

1. **Motion Analysis** study. Gravity: -Z for a vertical climb (pipe pointing up), -Y for a
   horizontal pipe.
2. **Springs**: one linear spring per slide, stiffness `rocker_spring_rate_n_mm`, free
   length chosen so the force at the outer stop equals the preload
   (S 23.0, M 57.4, L 217.6 N).
3. **Contact**: wheel tread solids <-> pipe inner face. Friction: static = mu_dry
   (steel 0.75), dynamic ~0.9 x that; for wet use 0.42. Material: "Rubber" on "Steel" with
   these overrides.
4. **Motors**:
   - expansion: rotary motor on the expansion motor shaft, *Distance* profile, run until the
     wheels touch, then switch to a torque limit (`calc_expansion_motor_torque_nm`) to
     squeeze;
   - drive: three rotary motors on the wheel axles, constant speed = v / (wheel radius),
     e.g. M at 95 mm/s: 95 / 22.5 = 4.2 rad/s.
5. Run 5 s at 200 frames/s. **Results** to plot and compare with the repository's numbers:

| Result | Should match (per size S / M / L) |
|---|---|
| Contact force per arm when gripped | working grip 28.0 / 65.4 / 233.6 N |
| Lever torque at the pivot | <= `calc_lever_torque_max_nm` 3.9 / 12.8 / 86.9 N m |
| Expansion motor torque while squeezing | <= `calc_expansion_motor_torque_nm` 0.11 / 0.20 / 0.84 N m |
| Body sag in a horizontal pipe | ~6 mm (see docs/real_world_issues.md, T5) |
| Slip in a vertical pipe with motors locked | 0 (self-locking worms) |

Tip: contact with many tread faces is slow. Replace each tread with a simple cylinder
(same diameter and width) for motion studies.

### 2.6 FEA (SolidWorks Simulation, static)

Use the design loads from `params_solidworks.txt` (`calc_*`). These cases are the ones the
hand calcs check; FEA should agree within ~20 % (stress concentrations aside):

| Part | Fixture | Load | Hand-calc result (S / M / L) |
|---|---|---|---|
| `lever_plate` (x2, use one plate with half the load) | Fixed geometry on the D-hole faces | at the rocker pin hole: radial force `calc_arm_structural_load_n` (43.8 / 99.9 / 344.2 N) + tangential traction 0.75 x the arm's working grip | bending stress 43.0 / 57.7 / 40.0 MPa (limit 120 MPa = 6061-T6 yield / 2.3); tip deflection 0.15 / 0.41 / 0.49 mm (limit 0.5) |
| `worm_sector` | Fixed on the D-bore | `calc_sector_tooth_force_n` (287 / 670 / 2373 N) on one tooth, at the pitch line | Lewis bending 86.9 / 84.6 / 89.9 MPa (limit 90 MPa, CuSn12) |
| `pivot_shaft` | Bearings as fixtures | lever torque `calc_lever_torque_max_nm` as a torque on the key flat | check torsion and the D-flat edge |
| `body_*_shell` | Fixed at the bulkhead ring | External pressure 0.1 MPa (10 m water, IP68 margin) | buckling factor > 3 |
| `wheel_hub` | Bearing seat fixed | half the arm structural load at the tread | |

Mesh: curvature-based, 4 Jacobian points, element size ~1/3 of the plate thickness at the
holes. Use **Design Insight** to see that the lightening slot in the lever stays in a
low-stress zone.

**Thermal study** (Simulation, steady state): heat power on each drive motor body =
copper loss 2 x R x I^2 (see `motors` in the YAML; e.g. M NEMA14 at 0.7 x rated current
~3 W each), convection on the outer hull 8 W/m^2 K at 40 C. Hottest motor should land near
the simulator's 70 C (T12).

---

## 3. Other CAD tools

| Tool | Import | Motion / FEA |
|---|---|---|
| **FreeCAD 1.0** (free) | File > Import `assembly.step`; Spreadsheet: import `params.csv` | Assembly workbench (joints) for motion; FEM workbench + CalculiX for the load cases in 2.6 |
| **Fusion 360** | Upload `assembly.step` | Joints (Revolute, Slider), Motion Study; Simulation > Static Stress; parameters via Change Parameters |
| **Onshape** | Import STEP, "Flatten" off | Mate connectors + Revolute/Slider mates; FEA via SimScale integration |
| **Ansys Mechanical / Discovery** | STEP | Same load cases; nonlinear contact for the tread |

---

## 4. Physics / robotics simulators (3-D contact, elbows, debris)

The repository's simulator is one-dimensional along the pipe (fast, runs the real firmware).
For 3-D effects (elbows, tilted entry, debris) build a rigid-body model:

| Tool | How |
|---|---|
| **Simscape Multibody** (MATLAB) | Install *Simscape Multibody Link* in SolidWorks, export the mated assembly (section 2.4) to XML, `smimport('PIPEBOT_M.xml')`. Add Spatial Contact Force blocks (wheel <-> pipe), Translational Spring on the slides, and drive the joints with the controller in Simulink. |
| **Gazebo (Harmonic) / ROS 2** | Make a URDF/SDF: links = body, 3 levers, 3 rockers, 6 wheels, camera pod; joints = 3 revolute levers (mimic one master joint for the synchronous crown), 3 revolute rockers, 3 prismatic slides with spring stiffness/damping, 6 continuous wheels. Visual meshes from `cad/exports/<size>/stl/`; collision = primitive cylinders. Pipe = mesh or a ring of boxes. Friction `mu` from the YAML. |
| **Webots** | Same structure as a PROTO; Webots' `Hinge2Joint` for the U-joint. |
| **MuJoCo** | MJCF with `equality` joints for the synchronous levers and `tendon`-free springs on slide joints; very fast contact for parameter sweeps. |
| **PyBullet** | Load the URDF; good for quick friction / slip studies in Python. |

Validate any of these against the repository's numbers before trusting it: grip force per
arm, slip = 0 when held, crawl speed vs command, sag in horizontal pipe.

---

## 5. What to compare, and where the numbers come from

| Quantity | Source |
|---|---|
| Every design check with its limit | `python calc/design_calcs.py` |
| Interference at 9 stroke positions | `cad/exports/<size>/interference.json` |
| Mass, CG | `docs/mass_report.md` |
| Behaviour in pipes (T1-T15) | `docs/acceptance_report.md`, raw CSVs in `tests/acceptance/out/` |
| Known problems and their fixes | `docs/real_world_issues.md` |
