// Connects the telemetry socket to the store and re-evaluates staleness 4x per second, so a
// frozen link is shown as stale even when no new message arrives.
import { useEffect, useState } from "react";

import { api } from "../api/client";
import { TelemetrySocket } from "../api/telemetrySocket";
import { isStale, useRobot } from "../state/robotStore";

export function useTelemetryConnection(): void {
  const setTelemetry = useRobot((s) => s.setTelemetry);
  const setSocketOpen = useRobot((s) => s.setSocketOpen);
  const setConfig = useRobot((s) => s.setConfig);
  useEffect(() => {
    const sock = new TelemetrySocket((t) => setTelemetry(t), setSocketOpen);
    sock.start();
    api.config().then(setConfig).catch(() => undefined);
    return () => sock.stop();
  }, [setTelemetry, setSocketOpen, setConfig]);
}

export function useStale(): boolean {
  const t = useRobot((s) => s.telemetry);
  const receivedAt = useRobot((s) => s.receivedAt);
  const open = useRobot((s) => s.socketOpen);
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 250);
    return () => clearInterval(id);
  }, []);
  return isStale(t, receivedAt, open, now);
}
