import { beforeEach, describe, expect, it } from "vitest";

import type { TelemetrySnapshot } from "../src/api/client";
import { STALE_MS, isStale, useRobot } from "../src/state/robotStore";

function tel(extra: Partial<TelemetrySnapshot> = {}): TelemetrySnapshot {
  return { age_ms: 10, stale: false, link: true, rtt_ms: 5, faults: [], ...extra } as TelemetrySnapshot;
}

describe("robotStore", () => {
  beforeEach(() => useRobot.setState({ telemetry: null, receivedAt: 0, pending: {}, dismissedFaults: [], history: [] }));

  it("is stale without telemetry, socket, link, or after STALE_MS", () => {
    expect(isStale(null, 0, true, 0)).toBe(true);
    expect(isStale(tel(), 1000, false, 1000)).toBe(true);
    expect(isStale(tel({ link: false }), 1000, true, 1000)).toBe(true);
    expect(isStale(tel({ stale: true }), 1000, true, 1000)).toBe(true);
    expect(isStale(tel(), 1000, true, 1000 + STALE_MS - 1)).toBe(false);
    expect(isStale(tel(), 1000, true, 1000 + STALE_MS + 1)).toBe(true);
  });

  it("keeps a command pending until telemetry confirms it", () => {
    const s = useRobot.getState();
    s.setPending("white", 60, 0);
    s.setTelemetry(tel({ light: { white_pct: 0, ir_pct: 0 } } as Partial<TelemetrySnapshot>), 10);
    expect(useRobot.getState().pending.white).toBeDefined();
    useRobot.getState().setTelemetry(tel({ light: { white_pct: 60, ir_pct: 0 } } as Partial<TelemetrySnapshot>), 20);
    expect(useRobot.getState().pending.white).toBeUndefined();
  });

  it("marks failures without dropping the pending entry", () => {
    useRobot.getState().setPending("pan", 30, 0);
    useRobot.getState().failPending("pan", "TIMEOUT");
    expect(useRobot.getState().pending.pan.failed).toBe("TIMEOUT");
  });

  it("re-shows a dismissed fault once it clears and comes back", () => {
    const s = useRobot.getState();
    s.setTelemetry(tel({ faults: ["GRIP_LOW"] } as Partial<TelemetrySnapshot>));
    s.dismissFaults(["GRIP_LOW"]);
    expect(useRobot.getState().dismissedFaults).toEqual(["GRIP_LOW"]);
    useRobot.getState().setTelemetry(tel({ faults: [] }));
    expect(useRobot.getState().dismissedFaults).toEqual([]);
  });

  it("records history with min grip and summed current", () => {
    useRobot.getState().setTelemetry(tel({ grip_force_n: [70, 60, 80], motor_current_a: [0.5, 0.5, 0.5, 0.2] } as Partial<TelemetrySnapshot>), 1000);
    const hist = useRobot.getState().history;
    const h = hist[hist.length - 1];
    expect(h.grip).toBe(60);
    expect(h.current).toBeCloseTo(1.7);
  });
});
