#include "sim/plant.h"

#include <algorithm>
#include <cmath>

#include "bot_params.h"

namespace sim {

namespace {
constexpr double kPi = 3.14159265358979;
constexpr double kDeg = kPi / 180.0;
constexpr double kBodyMu = 0.3;
constexpr double kKineticFrac = 0.85;
constexpr double kDriverRth = 1.6;    // K/W, driver + motor case to pod air (model)
constexpr double kDriverTau = 90.0;   // s
constexpr double kBatteryRint = 0.05; // ohm per cell
}  // namespace

Plant::Plant(const PlantConfig& cfg) : cfg_(cfg), rng_(cfg.seed ? cfg.seed : 1) {
  if (cfg_.sections.empty()) cfg_.sections.push_back(PipeSection{});
  x_m = cfg_.x0_m;
  theta = cfg_.theta0 < 0 ? params::THETA_MIN_RAD : cfg_.theta0;
  driver_temp_c = cfg_.ambient_c;
  soc = cfg_.battery_soc;
  tether_connected = cfg_.tether;
  extra_mass_kg = cfg_.extra_mass_kg;
  ambient_c = cfg_.ambient_c;
  if (cfg_.start_gripped) {
    // lever angle that puts the slides at mid-travel in the local pipe
    const double r_pipe = sectionAt(x_m).id_mm * 0.5;
    const double k_struct = 100.0 * params::SPRING_RATE_N_MM + 50.0;
    const double interf = params::SPRING_PRELOAD_N / k_struct + params::COMPLIANCE_TRAVEL_MM;
    const double s = (r_pipe + interf - params::WHEEL_DIA_MM * 0.5 - params::ROCKER_OFFSET_MM - params::PIVOT_R_MM) / params::LEVER_LEN_MM;
    theta = std::asin(std::clamp(s, -1.0, 1.0));
  }
}

double Plant::mass() const { return params::MASS_KG + extra_mass_kg; }

const PipeSection& Plant::sectionAt(double x) const {
  for (const auto& s : cfg_.sections)
    if (x >= s.start_m && x < s.end_m) return s;
  return cfg_.sections.back();
}

double Plant::contactRadiusNominal() const {
  return params::PIVOT_R_MM + params::LEVER_LEN_MM * std::sin(theta) + params::ROCKER_OFFSET_MM + params::WHEEL_DIA_MM * 0.5;
}

double Plant::noise(double sigma) {
  // Box-Muller on a 32-bit LCG (deterministic per seed)
  auto u = [this]() {
    rng_ = rng_ * 1664525u + 1013904223u;
    return (static_cast<double>(rng_ >> 8) + 0.5) / 16777216.0;
  };
  return sigma * cfg_.noise * std::sqrt(-2.0 * std::log(u())) * std::cos(2 * kPi * u());
}

double Plant::armForce(double interference, double& slide) const {
  const double k = params::SPRING_RATE_N_MM;
  const double p0 = params::SPRING_PRELOAD_N;
  const double travel = 2.0 * params::COMPLIANCE_TRAVEL_MM;
  const double k_struct = 100.0 * k + 50.0;
  slide = 0;
  if (interference <= 0) return 0;
  const double d0 = p0 / k_struct;
  if (interference <= d0) return k_struct * interference;
  const double s = interference - d0;
  if (s <= travel) {
    slide = s;
    return p0 + k * s;
  }
  slide = travel;
  return p0 + k * travel + k_struct * (s - travel);  // rocker bottomed on its stop
}

void Plant::solveContact(double r_pipe, double g_perp) {
  const double rc = contactRadiusNominal();
  const double body_gap = std::max(0.0, r_pipe - params::BODY_DIA_MM * 0.5);
  double ux[3], uy[3];
  for (int i = 0; i < 3; ++i) {
    const double a = (i * 120.0 + cfg_.roll_deg) * kDeg;
    ux[i] = std::cos(a);
    uy[i] = std::sin(a);
  }
  auto resid = [&](double dx, double dy, double& fx, double& fy) {
    fx = 0;
    fy = g_perp;  // sum F_i u_i must equal (0, -g_perp)  ->  residual = sum + (0, g_perp)
    for (int i = 0; i < 3; ++i) {
      double s;
      const double f = armForce(rc + dx * ux[i] + dy * uy[i] - r_pipe, s);
      fx += f * ux[i];
      fy += f * uy[i];
    }
    const double dn = std::hypot(dx, dy);
    if (dn > body_gap && dn > 1e-9) {  // body resting on the pipe wall
      const double fb = 2000.0 * (dn - body_gap);
      fx += fb * dx / dn;
      fy += fb * dy / dn;
    }
  };
  double dx = offset_x_mm, dy = offset_y_mm;
  for (int it = 0; it < 40; ++it) {
    double fx, fy;
    resid(dx, dy, fx, fy);
    if (std::hypot(fx, fy) < 1e-3) break;
    const double h = 1e-3;
    double fx1, fy1, fx2, fy2;
    resid(dx + h, dy, fx1, fy1);
    resid(dx, dy + h, fx2, fy2);
    const double a = (fx1 - fx) / h, b = (fx2 - fx) / h, c = (fy1 - fy) / h, d = (fy2 - fy) / h;
    const double det = a * d - b * c;
    if (std::fabs(det) < 1e-9) break;
    double sx = -(d * fx - b * fy) / det, sy = -(-c * fx + a * fy) / det;
    const double sn = std::hypot(sx, sy);
    if (sn > 2.0) {
      sx *= 2.0 / sn;
      sy *= 2.0 / sn;
    }
    dx += sx;
    dy += sy;
  }
  offset_x_mm = dx;
  offset_y_mm = dy;
  for (int i = 0; i < 3; ++i) arm[i].force_n = armForce(rc + dx * ux[i] + dy * uy[i] - r_pipe, arm[i].slide_mm);
}

void Plant::step(double dt) {
  t += dt;
  const PipeSection& sec = sectionAt(x_m);
  const double alpha = sec.incline_deg * kDeg;
  const double m = mass();
  const double g = params::GRAVITY;
  const bool powered = mcu_powered;

  // ---------------- expansion (self-locking: theta only moves when the motor turns it)
  exp_stalled = false;
  double omega = 0;
  if (powered && expand_enable_line && stepper_en[3]) omega = stepper_vel[3] / params::EXPANSION_USTEPS_PER_RAD;
  double lever_torque = 0;
  for (int i = 0; i < 3; ++i) lever_torque += arm[i].force_n * params::LEVER_LEN_MM * std::cos(theta) * 1e-3;
  if (omega > 0) {
    const double need = lever_torque / (params::EXPANSION_RATIO * params::EXP_CHAIN_EFF);
    const double avail = params::EXP_HOLDING_NM * params::SQUEEZE_TORQUE_FRAC * current_scale[3];
    if (need > avail) exp_stalled = true;
  }
  const double th_lo = params::THETA_MIN_RAD - 1.0 * kDeg, th_hi = params::THETA_MAX_RAD + 1.0 * kDeg;
  if ((omega > 0 && theta >= th_hi) || (omega < 0 && theta <= th_lo)) exp_stalled = true;
  if (!exp_stalled) theta = std::clamp(theta + omega * dt, th_lo, th_hi);

  // ---------------- contact + body sag
  solveContact(sec.id_mm * 0.5, m * g * std::cos(alpha));
  double sum_f = 0;
  for (auto& a : arm) sum_f += a.force_n;

  // ---------------- drive: stick/slip axial dynamics
  const double mu = (sec.wet || wet_override) ? params::MU_WET : params::MU_DRY;
  const double r_wheel = params::WHEEL_DIA_MM * 0.5e-3;
  double u[3];
  for (int i = 0; i < 3; ++i) {
    const bool on = powered && drive_enable_line && stepper_en[i] && !arm[i].stalled;
    u[i] = on ? stepper_vel[i] / params::DRIVE_USTEPS_PER_MM : 0.0;  // mm/s
    arm[i].surface_mm_s = u[i];
  }
  const double gx = -m * g * std::sin(alpha);
  double ft = -pull_force_n;
  if (cfg_.tether && tether_connected && v_m_s >= 0) ft -= cfg_.tether_drag_n_per_m * std::max(0.0, x_m);
  const double ext = gx + ft;
  double u_eff = 0;
  if (sum_f > 1e-6) {
    for (int i = 0; i < 3; ++i) u_eff += arm[i].force_n * u[i];
    u_eff = u_eff / sum_f * 1e-3;  // m/s
  }
  double traction = 0;
  if (sum_f > 1e-6) {
    const double cap = mu * sum_f;
    const double req = m * (u_eff - v_m_s) / dt - ext;
    if (std::fabs(req) <= cap) {
      v_m_s = u_eff;
      slipping = false;
      traction = req;
    } else {
      traction = (req > 0 ? 1 : -1) * kKineticFrac * cap;
      v_m_s += (traction + ext) / m * dt;
      slipping = true;
    }
  } else {
    // lying on the pipe: body friction only
    const double cap = kBodyMu * m * g * std::cos(alpha);
    if (std::fabs(ext) <= cap && std::fabs(v_m_s) < 1e-3) {
      v_m_s = 0;
    } else {
      v_m_s += (ext - (v_m_s > 0 ? 1 : -1) * kKineticFrac * cap) / m * dt;
    }
    slipping = std::fabs(v_m_s) > 1e-4;
  }
  // motor stall: wheel torque demand vs stepper torque at speed through the worm
  for (int i = 0; i < 3; ++i) {
    if (arm[i].stalled) {
      arm[i].stalled = false;  // retry next step (missed steps, not a latch)
      continue;
    }
    if (u[i] == 0.0 || sum_f <= 1e-6) continue;
    const double rpm = std::fabs(u[i]) / (kPi * params::WHEEL_DIA_MM) * 60.0 * params::DRIVE_WORM_RATIO;
    const double t_motor = params::DRIVE_HOLDING_NM * current_scale[i] * std::max(0.05, 1.0 - 0.6 * rpm / params::DRIVE_MAX_RPM);
    const double t_avail = t_motor * params::DRIVE_WORM_RATIO * params::DRIVE_WORM_EFF;
    const double t_need = std::fabs(traction) * (arm[i].force_n / sum_f) * r_wheel;
    if (t_need > t_avail) arm[i].stalled = true;
  }
  for (int i = 0; i < 3; ++i) {
    arm[i].wheel_angle = std::fmod(arm[i].wheel_angle + u[i] * 1e-3 / r_wheel * dt + 2 * kPi * 1000, 2 * kPi);
  }
  x_m += v_m_s * dt;
  if (x_m < 0) {  // pipe bottom / entry floor
    x_m = 0;
    v_m_s = 0;
  }
  const bool commanded_still = u[0] == 0 && u[1] == 0 && u[2] == 0;
  if (commanded_still) {
    if (!hold_ref_set_) {
      hold_ref_x_ = x_m;
      hold_ref_set_ = true;
    }
    max_slip_mm = std::max(max_slip_mm, std::fabs(x_m - hold_ref_x_) * 1000.0);
  } else {
    hold_ref_set_ = false;
  }

  // ---------------- power
  double p_drive = 0, p_exp = 0;
  for (int i = 0; i < 3; ++i) {
    if (!(powered && drive_enable_line && stepper_en[i])) continue;
    const double cur = params::DRIVE_RATED_A * current_scale[i] * (u[i] != 0 ? 0.7 : params::HOLD_CURRENT_FRAC);
    p_drive += 2 * params::DRIVE_WINDING_OHM * cur * cur;
    if (sum_f > 1e-6) p_drive += std::fabs(traction * v_m_s) * (arm[i].force_n / sum_f) / (params::DRIVE_WORM_EFF * 0.4);
  }
  if (powered && expand_enable_line && stepper_en[3] && omega != 0) {
    const double cur = params::EXP_RATED_A * 0.7 * current_scale[3];
    p_exp = 2 * params::EXP_WINDING_OHM * cur * cur + std::fabs(lever_torque * omega) / std::max(0.05, static_cast<double>(params::EXP_CHAIN_EFF));
  }
  drive_a = p_drive / params::MOTOR_RAIL_V;
  expand_a = p_exp / params::MOTOR_RAIL_V;
  const double p_elec = (powered ? params::ELECTRONICS_W + 0.5 + params::LED_WHITE_W * white / 100.0 + params::LED_IR_W * ir / 100.0 : 0) + p_drive + p_exp;
  const double p_in = p_elec / params::CONVERTER_EFF;
  const double cells = params::BATTERY_SERIES;
  battery_v = cells > 0 ? cells * (3.3 + 0.9 * soc) : 0.0;
  if (cfg_.tether && tether_connected) {
    const double r_loop = params::TETHER_LOOP_OHM_PER_M * (std::max(0.0, x_m) + 5.0);
    const double vt = params::TETHER_V;
    const double disc = vt * vt - 4 * p_in * r_loop;
    bus_v = disc > 0 ? (vt + std::sqrt(disc)) / 2 : 0.0;
    on_battery = false;
  } else if (cells > 0 && soc > 0) {
    const double i_bat = p_in / std::max(1.0, battery_v);
    soc = std::max(0.0, soc - i_bat * dt / 3600.0 / (params::CELL_AH * params::BATTERY_PARALLEL));
    battery_v = cells * (3.3 + 0.9 * soc) - i_bat * kBatteryRint * cells / std::max(1, params::BATTERY_PARALLEL);
    bus_v = battery_v;
    on_battery = true;
  } else {
    bus_v = 0;
    on_battery = false;
  }
  mcu_powered = bus_v > 9.0;
  driver_temp_c += ((ambient_c + kDriverRth * (p_drive + p_exp)) - driver_temp_c) / kDriverTau * dt;

  // ---------------- gimbal
  if (powered) {
    const double rate = 120.0 * dt;
    double dp = pan_cmd - pan;
    pan += std::clamp(dp, -rate, rate);
    tilt += std::clamp(std::clamp(tilt_cmd, -90.0, 90.0) - tilt, -rate, rate);
  }
}

}  // namespace sim
