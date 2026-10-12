export function fmt(v: number | null | undefined, digits = 1, unit = ""): string {
  if (v === null || v === undefined || Number.isNaN(v)) return "--";
  return `${v.toFixed(digits)}${unit ? " " + unit : ""}`;
}

export function mmToIn(mm: number): number {
  return mm / 25.4;
}

export const SEVERE_FAULTS = ["ESTOP", "OVERCURRENT", "OVERTEMP", "EXPANSION_STALL", "ENCODER_FAIL", "CRITICAL_BATTERY", "SUPPLY_UNDERVOLT", "GRIP_LOW"];

export const FAULT_TEXT: Record<string, string> = {
  SLIP_WARNING: "Wheel speed not following command (stall) - drive stopped",
  GROUND_SLIP: "Wheels slipping on the wall - odometry unreliable",
  OVERCURRENT: "Over-current - motors stopped",
  OVERTEMP: "Driver over-temperature - motors stopped",
  THERMAL_DERATE: "Hot: speed and current reduced",
  LOW_BATTERY: "Battery low",
  CRITICAL_BATTERY: "Battery critical - holding position",
  SUPPLY_UNDERVOLT: "Tether supply too low - holding position",
  HEARTBEAT_LOST: "No commands from the Pi - stopped, still gripping",
  ESTOP: "E-STOP active",
  EXPANSION_STALL: "Arms jammed - expansion stopped",
  GRIP_LOW: "Grip below safe level - re-gripping",
  NO_CONTACT: "Arms reached full reach without touching the wall",
  IMU_FAIL: "IMU not responding",
  TOF_FAIL: "Range sensor not responding",
  ENCODER_FAIL: "Lever/wheel encoder failure",
  WATCHDOG_RESET: "Controller rebooted (watchdog)",
  COMMAND_STALE: "Command expired (TTL) - stopped",
};
