#include "control/expansion_loop.h"

#include <cmath>

#include "bot_params.h"
#include "control/kinematics.h"
#include "control/pid.h"

namespace ctl {

namespace {
constexpr float kAngleTol = 0.2f * kin::kDeg;
constexpr float kDiameterTolMm = 0.3f;  // DIAMETER mode stops within this (T4: +/-1 mm)
// Lever angle = stepper dead-reckoning (600:1+ to the lever, so microsteps are microradians)
// pulled toward the absolute encoder with this weight per tick (tau ~0.25 s at 200 Hz). The
// AS5600 alone is ~0.09 deg per count: 0.75 mm of diameter on the 250 mm L lever.
constexpr float kEncBlend = 0.02f;
constexpr float kTwoPi = 6.28318531f;
}  // namespace

// Lever geometry gives the wheels' reach with the rockers on their outer stops; a gripping
// wheel sits `slide` mm inside that, plus its tread / frame squash F / k_contact. The mean
// over the three arms cancels a sideways body offset (arms at 120 deg), so it measures the
// pipe, not the sag.
float ExpansionLoop::diameter() const {
  const float reach = kin::diameterForTheta(theta_);
  if (contactCount() < 2) return reach;  // not gripping: report the wheels' reach
  float inset = 0;
  for (int i = 0; i < 3; ++i) inset += slide_[i] + grip_[i] / params::CONTACT_STIFFNESS_N_MM;
  return reach - 2.0f * inset / 3.0f;
}

int ExpansionLoop::contactCount() const {
  int n = 0;
  for (float s : slide_) n += s > params::CONTACT_SLIDE_MM ? 1 : 0;
  return n;
}

// Force on an arm for control purposes. The hall sensor only sees force above the spring
// preload. In a horizontal pipe the body sags and the top arm drops below preload while
// still touching: if the other two arms are loaded it is counted at preload (upper bound).
float ExpansionLoop::effForce(int i) const {
  if (slide_[i] > params::CONTACT_SLIDE_MM) return grip_[i];
  return contactCount() >= 2 ? params::SPRING_PRELOAD_N : 0.0f;
}

float ExpansionLoop::gripMin() const { return std::fmin(effForce(0), std::fmin(effForce(1), effForce(2))); }

float ExpansionLoop::gripAvg() const { return (effForce(0) + effForce(1) + effForce(2)) / 3.0f; }

float ExpansionLoop::gripSqueeze() const {
  const float a = grip_[0], b = grip_[1], c = grip_[2];
  return std::fmax(std::fmin(a, b), std::fmin(std::fmax(a, b), c));
}

// Gripped = at least two arms loaded (the third is then held against the wall by geometry).
bool ExpansionLoop::allContact() const { return contactCount() >= 2; }

void ExpansionLoop::clearFaults() {
  stall_ = over_current_ = no_contact_ = false;
  stall_s_ = oc_s_ = 0;
}

void ExpansionLoop::drive(const hal::Hardware& hw, float omega_target, float dt, float current_scale) {
  omega_ = slew(omega_, omega_target, params::EXPANSION_ACCEL_RAD_S2, dt);  // soft start / stop
  if (std::fabs(omega_) < 1e-6f && omega_target == 0.0f) omega_ = 0.0f;
  const bool want = omega_ != 0.0f;
  if (want != enabled_) {
    hw.stepper[3]->setCurrent((current_a_ > 0 ? current_a_ : params::EXPANSION_CURRENT_A) * current_scale, 0.0f);
    hw.stepper[3]->enable(want);
    hw.safety->expandEnable(want);
    enabled_ = want;
  }
  hw.stepper[3]->setVelocity(omega_ * params::EXPANSION_USTEPS_PER_RAD);
}

void ExpansionLoop::step(const hal::Hardware& hw, const ExpCmd& cmd, float dt) {
  // --- sense
  float a = 0;
  enc_ok_ = hw.lever_enc->read(a);
  if (enc_ok_) {
    if (a > kTwoPi * 0.5f) a -= kTwoPi;  // lever never passes +/-180 deg
    enc_theta_ = a;
  }
  if (!theta_init_) {
    theta_ = enc_theta_;
    theta_init_ = enc_ok_;
  } else {
    theta_ += omega_ * dt;  // the stepper turned exactly as commanded unless it stalled
    if (enc_ok_) theta_ += kEncBlend * (enc_theta_ - theta_);
  }
  for (int i = 0; i < 3; ++i) {
    float s = 0;
    if (hw.sensors->gripSlideMm(i, s)) slide_[i] = s;
    grip_[i] = kin::gripForce(slide_[i]);
  }
  const hal::PowerData pw = hw.power->read();
  oc_s_ = (pw.expand_a > params::EXP_OVER_CURRENT_A) ? oc_s_ + dt : 0.0f;
  if (oc_s_ > 0.05f) over_current_ = true;

  if (cmd.mode != last_mode_) {
    done_ = false;
    no_contact_ = false;
    maintain_active_ = false;
    last_mode_ = cmd.mode;
  }
  const float wmax = params::EXPANSION_MAX_RAD_S;
  float w = 0;
  const bool fault = over_current_ || stall_ || !enc_ok_;
  if (!fault) {
    switch (cmd.mode) {
      case ExpMode::LOCKED:
        done_ = true;
        break;
      case ExpMode::DIAMETER:
      case ExpMode::RETRACT: {
        const float tgt = cmd.mode == ExpMode::RETRACT ? params::THETA_MIN_RAD : kin::thetaForDiameter(cmd.target);
        const float err = tgt - theta_;
        // d(diameter)/d(theta) = 2 L cos(theta): convert the diameter tolerance to an angle here
        const float tol = cmd.mode == ExpMode::RETRACT ? kAngleTol : kDiameterTolMm / (2.0f * params::LEVER_LEN_MM * std::cos(theta_));
        if (std::fabs(err) < tol) {
          done_ = true;
        } else {
          // proportional approach with a speed cap (accel-limited in drive())
          w = clampf(err * 4.0f, -wmax, wmax);
          if (std::fabs(w) < 0.05f * wmax) w = err > 0 ? 0.05f * wmax : -0.05f * wmax;
        }
        // never crush: stop extending once the squeeze reaches the force limit
        if (w > 0 && gripSqueeze() > params::GRIP_LIMIT_N) {
          w = 0;
          done_ = true;
        }
        break;
      }
      case ExpMode::FORCE:
      case ExpMode::MAINTAIN: {
        const float tgt = clampf(cmd.target, params::SPRING_PRELOAD_N + 1.0f, params::GRIP_LIMIT_N);
        const float squeeze = gripSqueeze();
        if (cmd.mode == ExpMode::MAINTAIN) {
          // only act when out of band for a while (worms stay locked otherwise)
          const bool low = !allContact() || gripAvg() < tgt - params::REGRIP_DEADBAND_N || gripMin() < params::GRIP_LOW_N;
          const bool high = squeeze > params::GRIP_LIMIT_N;
          maintain_s_ = (low || high) ? maintain_s_ + dt : 0.0f;
          if (maintain_s_ > 0.3f) maintain_active_ = true;
          if (!maintain_active_) {
            done_ = true;
            break;
          }
        }
        if (!allContact()) {
          if (theta_ >= params::THETA_MAX_RAD - kAngleTol) {
            no_contact_ = true;
            done_ = true;
            maintain_active_ = false;
          } else {
            w = wmax;  // approach at full speed; the force servo takes over at contact
          }
        } else {
          const float err = tgt - gripAvg();
          if (squeeze > params::GRIP_LIMIT_N) {
            w = -0.1f * wmax;
          } else if (std::fabs(err) < 0.5f * params::REGRIP_DEADBAND_N) {
            done_ = true;
            maintain_active_ = false;
          } else {
            w = clampf(k_force_ * err, -0.3f * wmax, 0.3f * wmax);
          }
        }
        if (theta_ <= params::THETA_MIN_RAD && w < 0) w = 0;
        if (theta_ >= params::THETA_MAX_RAD && w > 0) w = 0;
        break;
      }
    }
  }
  drive(hw, w, dt, cmd.current_scale);

  // --- stall: motor turning but the lever not following (missed steps / jam)
  if (have_prev_ && dt > 0 && std::fabs(omega_) > 0.3f * wmax) {
    const float rate = (enc_theta_ - prev_enc_) / dt;  // raw encoder: the fused angle would hide a stall
    const bool lag = std::fabs(rate) < 0.2f * std::fabs(omega_);
    stall_s_ = lag ? stall_s_ + dt : 0.0f;
  } else {
    stall_s_ = 0;
  }
  if (stall_s_ > 0.3f || hw.stepper[3]->status().stalled) {
    stall_ = true;
    omega_ = 0;
    hw.stepper[3]->setVelocity(0);
  }
  prev_enc_ = enc_theta_;
  have_prev_ = enc_ok_;
}

}  // namespace ctl
