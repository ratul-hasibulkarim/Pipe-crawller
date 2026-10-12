// Robot state machine. Pure logic (no hardware), unit-tested in test/test_state_machine.
//
//   IDLE --expand--> EXPANDING --done+contact--> GRIPPED --drive--> CRAWLING
//                        |  done, no contact -> IDLE        ^           | stop / TTL / heartbeat lost
//                        v                                  |           v
//   IDLE <--done-- RETRACTING <--retract-- GRIPPED/HOLDING <--drive-- HOLDING --10 s idle--> GRIPPED
//   any --fault/estop--> FAULT --clear--> HOLDING (if gripped) | IDLE
//   any --undervolt--> LOW_POWER_SAFE --recovered--> HOLDING (if gripped) | IDLE
//
// The grip is never released automatically: FAULT, LOW_POWER_SAFE and heartbeat loss
// all stop the drive and leave the self-locking expansion where it is.
#pragma once
#include <cstdint>

#include "protocol_gen.h"

namespace ctl {

using proto::State;

class StateMachine {
 public:
  State state() const { return s_; }
  bool gripped() const { return gripped_; }
  uint32_t enteredMs() const { return entered_ms_; }
  // Each returns true if the request was accepted.
  bool expand(bool retract, uint32_t now);
  void expansionDone(bool all_contact, uint32_t now);
  void expansionStale(bool all_contact, uint32_t now);
  bool drive(bool moving, uint32_t now);
  void holdTimeout(uint32_t now, uint32_t hold_ms);
  void fault(uint32_t now);
  bool clearFault(bool conditions_ok, uint32_t now);
  void lowPower(bool low, uint32_t now);
  // After an MCU reset inside a pipe: the lever sensors show all arms loaded.
  void assumeGripped(uint32_t now);
  // Change notification: returns true once per transition.
  bool takeChanged(State& from, State& to);

 private:
  State s_ = State::IDLE;
  State prev_ = State::IDLE;
  bool gripped_ = false;
  bool changed_ = false;
  uint32_t entered_ms_ = 0;
  void go(State to, uint32_t now);
  State restState() const { return gripped_ ? State::HOLDING : State::IDLE; }
};

}  // namespace ctl
