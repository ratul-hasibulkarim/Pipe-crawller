// Physics model of PIPEBOT-X in a pipe, for the software-in-the-loop simulator and
// the native tests. Quasi-static arm/spring contact with body sag, axial dynamics with
// stick/slip friction, self-locking worms (no back-drive with or without power),
// stepper torque/speed limits, supply (tether + battery), driver heating, sensors.
// All constants come from bot_params.h (same source as the firmware).
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace sim {

struct PipeSection {
  double start_m = 0, end_m = 1e9;
  double id_mm = 150;
  double incline_deg = 0;   // + = pipe rises toward +x
  bool wet = false;
  double water_depth_m = 0; // robot submerged by this much (pressure sensor)
  double gas_ppm = 0;
};

struct PlantConfig {
  std::vector<PipeSection> sections;
  double pipe_end_m = 1e9;       // ToF target ahead (pipe end / obstacle)
  double x0_m = 0.5;             // start position of the robot's rear face
  double theta0 = -1;            // start lever angle, <0 = fully retracted
  bool start_gripped = false;    // placed in the pipe already gripping (springs at mid-travel)
  double roll_deg = 0;           // robot roll about the pipe axis (arm A vs gravity)
  bool tether = true;            // powered over the tether
  double ambient_c = 25;
  double tether_drag_n_per_m = 0.05;
  double extra_mass_kg = 0;      // T1 hang test
  double battery_soc = 1.0;
  double noise = 1.0;            // sensor noise multiplier
  uint32_t seed = 1;
};

struct ArmTruth {
  double force_n = 0, slide_mm = 0;
  double wheel_angle = 0;        // rad, what the AS5600 sees
  double surface_mm_s = 0;       // wheel surface speed
  bool stalled = false;
};

class Plant {
 public:
  explicit Plant(const PlantConfig& cfg);

  // actuators (written by the simulated HAL)
  double stepper_vel[4] = {0, 0, 0, 0};  // usteps/s
  bool stepper_en[4] = {false, false, false, false};
  double current_scale[4] = {1, 1, 1, 1};
  bool drive_enable_line = true, expand_enable_line = true;
  double pan_cmd = 0, tilt_cmd = 0;
  int white = 0, ir = 0;

  // events
  bool tether_connected = true;
  double pull_force_n = 0;       // external backward pull (tether yank)
  bool wet_override = false;
  bool estop_loop_open = false;
  double extra_mass_kg = 0;      // hang test load (T1) - changeable at run time
  double ambient_c = 25;

  void step(double dt);

  // truth
  double t = 0, x_m = 0, v_m_s = 0, theta = 0;
  double offset_x_mm = 0, offset_y_mm = 0;
  ArmTruth arm[3];
  bool slipping = false;
  bool exp_stalled = false;
  double pan = 0, tilt = 0;
  double driver_temp_c = 25;
  double bus_v = 48, battery_v = 0, soc = 1, drive_a = 0, expand_a = 0;
  bool mcu_powered = true;
  bool on_battery = false;
  double max_slip_mm = 0;        // largest uncommanded displacement seen (hold tests)

  const PipeSection& sectionAt(double x) const;
  double pipeIdAt(double x) const { return sectionAt(x).id_mm; }
  double mass() const;
  double contactRadiusNominal() const;  // contact radius with slides at their stops
  double noise(double sigma);
  const PlantConfig& cfg() const { return cfg_; }

 private:
  PlantConfig cfg_;
  uint32_t rng_;
  double hold_ref_x_ = 0;
  bool hold_ref_set_ = false;
  void solveContact(double r_pipe_mm, double g_perp_n);
  double armForce(double interference_mm, double& slide) const;
};

}  // namespace sim
