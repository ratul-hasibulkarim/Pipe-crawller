#include "control/state_machine.h"

namespace ctl {

void StateMachine::go(State to, uint32_t now) {
  if (to == s_) return;
  prev_ = s_;
  s_ = to;
  changed_ = true;
  entered_ms_ = now;
  if (to == State::GRIPPED || to == State::CRAWLING || to == State::HOLDING) gripped_ = true;
  if (to == State::IDLE) gripped_ = false;
}

bool StateMachine::takeChanged(State& from, State& to) {
  if (!changed_) return false;
  changed_ = false;
  from = prev_;
  to = s_;
  return true;
}

bool StateMachine::expand(bool retract, uint32_t now) {
  switch (s_) {
    case State::IDLE:
    case State::GRIPPED:
    case State::HOLDING:
    case State::EXPANDING:
    case State::RETRACTING:
      go(retract ? State::RETRACTING : State::EXPANDING, now);
      return true;
    default:
      return false;  // CRAWLING (stop first), FAULT, LOW_POWER_SAFE
  }
}

void StateMachine::expansionDone(bool all_contact, uint32_t now) {
  if (s_ == State::EXPANDING) {
    if (all_contact) {
      go(State::GRIPPED, now);
    } else {
      go(State::IDLE, now);
    }
  } else if (s_ == State::RETRACTING) {
    go(State::IDLE, now);
  }
}

void StateMachine::expansionStale(bool all_contact, uint32_t now) {
  if (s_ == State::EXPANDING || s_ == State::RETRACTING) go(all_contact ? State::GRIPPED : State::IDLE, now);
}

bool StateMachine::drive(bool moving, uint32_t now) {
  switch (s_) {
    case State::GRIPPED:
    case State::HOLDING:
      if (moving) go(State::CRAWLING, now);
      return true;
    case State::CRAWLING:
      if (!moving) go(State::HOLDING, now);
      return true;
    default:
      return !moving;  // a zero-speed drive is always harmless
  }
}

void StateMachine::holdTimeout(uint32_t now, uint32_t hold_ms) {
  if (s_ == State::HOLDING && now - entered_ms_ > hold_ms) go(State::GRIPPED, now);
}

void StateMachine::fault(uint32_t now) { go(State::FAULT, now); }

bool StateMachine::clearFault(bool conditions_ok, uint32_t now) {
  if (s_ != State::FAULT) return true;
  if (!conditions_ok) return false;
  go(restState(), now);
  return true;
}

void StateMachine::lowPower(bool low, uint32_t now) {
  if (low) {
    if (s_ != State::FAULT) go(State::LOW_POWER_SAFE, now);
  } else if (s_ == State::LOW_POWER_SAFE) {
    go(restState(), now);
  }
}

void StateMachine::assumeGripped(uint32_t now) {
  if (s_ == State::IDLE) go(State::HOLDING, now);
}

}  // namespace ctl
