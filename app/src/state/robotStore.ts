// Global robot state (zustand). Latency-honest: commanded values stay "pending" until the
// robot's telemetry confirms them; fail-visible: `isStale` drives the desaturated UI.
import { create } from "zustand";

import type { RobotConfig, TelemetrySnapshot } from "../api/client";

export const STALE_MS = 500;

export interface Pending {
  value: number;
  sentAt: number;
  failed?: string;
}

export interface HistoryPoint {
  t: number;
  distance: number;
  orientation: number;
  grip: number;
  current: number;
  diameter: number;
}

export interface Settings {
  hud: boolean;
  units: "metric" | "imperial";
  operator: string;
}

interface RobotState {
  telemetry: TelemetrySnapshot | null;
  receivedAt: number;
  socketOpen: boolean;
  config: RobotConfig | null;
  pending: Record<string, Pending>;
  dismissedFaults: string[];
  history: HistoryPoint[];
  markers: { t: number; label: string }[];
  settings: Settings;
  recordingId: string | null;
  setTelemetry: (t: TelemetrySnapshot, now?: number) => void;
  setSocketOpen: (open: boolean) => void;
  setConfig: (c: RobotConfig) => void;
  setPending: (key: string, value: number, now?: number) => void;
  failPending: (key: string, reason: string) => void;
  clearPending: (key: string) => void;
  dismissFaults: (faults: string[]) => void;
  addMarker: (label: string) => void;
  setSettings: (s: Partial<Settings>) => void;
  setRecording: (id: string | null) => void;
}

/** Robot-side value used to confirm each pending command. */
export function confirmedValue(t: TelemetrySnapshot | null, key: string): number | undefined {
  if (!t) return undefined;
  switch (key) {
    case "white":
      return t.light?.white_pct;
    case "ir":
      return t.light?.ir_pct;
    case "pan":
      return t.gimbal?.pan_deg;
    case "tilt":
      return t.gimbal?.tilt_deg;
    case "diameter":
      return t.est_diameter_mm;
    default:
      return undefined;
  }
}

const TOLERANCE: Record<string, number> = { white: 0.5, ir: 0.5, pan: 2, tilt: 2, diameter: 3 };

export const useRobot = create<RobotState>((set, get) => ({
  telemetry: null,
  receivedAt: 0,
  socketOpen: false,
  config: null,
  pending: {},
  dismissedFaults: [],
  history: [],
  markers: [],
  settings: { hud: true, units: "metric", operator: "" },
  recordingId: null,
  setTelemetry: (t, now = Date.now()) => {
    const pending = { ...get().pending };
    for (const [k, p] of Object.entries(pending)) {
      const v = confirmedValue(t, k);
      if (v !== undefined && Math.abs(v - p.value) <= (TOLERANCE[k] ?? 0.5)) delete pending[k];
    }
    // faults that cleared may be shown again next time they appear
    const active = t.faults ?? [];
    const dismissed = get().dismissedFaults.filter((f) => active.includes(f as never));
    const hist = get().history;
    const point: HistoryPoint = {
      t: now / 1000,
      distance: t.distance_m ?? 0,
      orientation: t.pipe_orientation_deg ?? 0,
      grip: t.grip_force_n ? Math.min(...t.grip_force_n) : 0,
      current: t.motor_current_a ? t.motor_current_a.reduce((a, b) => a + b, 0) : 0,
      diameter: t.est_diameter_mm ?? 0,
    };
    const history = hist.length > 1200 ? [...hist.slice(-1200), point] : [...hist, point];
    set({ telemetry: t, receivedAt: now, pending, dismissedFaults: dismissed, history });
  },
  setSocketOpen: (open) => set({ socketOpen: open }),
  setConfig: (c) => set({ config: c }),
  setPending: (key, value, now = Date.now()) => set({ pending: { ...get().pending, [key]: { value, sentAt: now } } }),
  failPending: (key, reason) => {
    const p = get().pending[key];
    if (p) set({ pending: { ...get().pending, [key]: { ...p, failed: reason } } });
  },
  clearPending: (key) => {
    const pending = { ...get().pending };
    delete pending[key];
    set({ pending });
  },
  dismissFaults: (faults) => set({ dismissedFaults: Array.from(new Set([...get().dismissedFaults, ...faults])) }),
  addMarker: (label) => set({ markers: [...get().markers, { t: Date.now() / 1000, label }] }),
  setSettings: (s) => set({ settings: { ...get().settings, ...s } }),
  setRecording: (id) => set({ recordingId: id }),
}));

/** True when nothing the UI shows can be trusted as live. */
export function isStale(t: TelemetrySnapshot | null, receivedAt: number, socketOpen: boolean, now = Date.now()): boolean {
  if (!t || !socketOpen) return true;
  if (t.stale || !t.link) return true;
  return now - receivedAt > STALE_MS;
}
