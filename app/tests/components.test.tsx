import { act, cleanup, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import type { TelemetrySnapshot } from "../src/api/client";
import { EStopButton, HOLD_MS } from "../src/components/EStopButton";
import { FaultBanner } from "../src/components/FaultBanner";
import { useRobot } from "../src/state/robotStore";

function setFaults(faults: string[]): void {
  useRobot.setState({ telemetry: { age_ms: 0, stale: false, link: true, rtt_ms: 1, faults } as unknown as TelemetrySnapshot, dismissedFaults: [] });
}

describe("EStopButton", () => {
  beforeEach(() => {
    vi.useFakeTimers({ toFake: ["requestAnimationFrame", "cancelAnimationFrame", "performance", "setTimeout"] });
    setFaults([]);
  });
  afterEach(() => {
    cleanup();
    vi.useRealTimers();
  });

  it("does not fire on a short tap", async () => {
    const onTrigger = vi.fn().mockResolvedValue({ status: "ESTOPPED" });
    render(<EStopButton onTrigger={onTrigger} />);
    const b = screen.getByTestId("estop");
    fireEvent.pointerDown(b);
    await act(async () => vi.advanceTimersByTime(HOLD_MS * 0.6));
    fireEvent.pointerUp(b);
    await act(async () => vi.advanceTimersByTime(HOLD_MS));
    expect(onTrigger).not.toHaveBeenCalled();
  });

  it("fires once after a full long-press", async () => {
    const onTrigger = vi.fn().mockResolvedValue({ status: "ESTOPPED" });
    render(<EStopButton onTrigger={onTrigger} />);
    const b = screen.getByTestId("estop");
    fireEvent.pointerDown(b);
    await act(async () => vi.advanceTimersByTime(HOLD_MS + 50));
    fireEvent.pointerUp(b);
    expect(onTrigger).toHaveBeenCalledTimes(1);
  });

  it("shows RETRY when the robot does not confirm", async () => {
    const onTrigger = vi.fn().mockResolvedValue({ status: "TIMEOUT" });
    render(<EStopButton onTrigger={onTrigger} />);
    fireEvent.pointerDown(screen.getByTestId("estop"));
    await act(async () => vi.advanceTimersByTime(HOLD_MS + 50));
    expect(screen.getByTestId("estop").textContent).toContain("RETRY");
  });

  it("offers clear while ESTOP is latched", async () => {
    setFaults(["ESTOP"]);
    const onClear = vi.fn().mockResolvedValue({ status: "OK" });
    render(<EStopButton onClear={onClear} />);
    expect(screen.getByTestId("estop").textContent).toContain("HOLD TO CLEAR");
    fireEvent.pointerDown(screen.getByTestId("estop"));
    await act(async () => vi.advanceTimersByTime(HOLD_MS + 50));
    expect(onClear).toHaveBeenCalledTimes(1);
  });
});

describe("FaultBanner", () => {
  afterEach(cleanup);

  it("renders nothing without faults", () => {
    setFaults([]);
    const { container } = render(<FaultBanner />);
    expect(container.firstChild).toBeNull();
  });

  it("shows faults and hides them on dismiss", () => {
    setFaults(["GRIP_LOW", "TETHER_LOW"]);
    render(<FaultBanner />);
    expect(screen.getByRole("alert").textContent).toContain("GRIP_LOW");
    fireEvent.click(screen.getByLabelText("Dismiss faults"));
    expect(screen.queryByRole("alert")).toBeNull();
  });
});
