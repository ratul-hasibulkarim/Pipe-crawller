// RP2040 entry point. Core 0 runs the control core at 1 kHz; core 1 polls slow sensors.
// The timer ISR only sets a flag (well under the 10 us ISR budget); nothing prints from an ISR.
#ifdef ARDUINO
#include <Arduino.h>
#include <pico/time.h>

#include "core/robot.h"
#include "hal/rp2040/rp2040_hal.h"

namespace {
volatile bool g_tick_due = false;
repeating_timer_t g_timer;

bool onTick(repeating_timer_t*) {
  g_tick_due = true;
  return true;
}

core::Robot& robot() {
  static core::Robot r(rp2040::board().hardware());  // static storage, no heap
  return r;
}
}  // namespace

void setup() {
  rp2040::board().begin();
  robot().begin(micros());
  add_repeating_timer_us(-1000, onTick, nullptr, &g_timer);
}

void loop() {
  if (g_tick_due) {
    g_tick_due = false;
    robot().tick(micros());
  }
}

void setup1() { rp2040::board().beginCore1(); }

void loop1() { rp2040::board().pollCore1(); }
#endif
