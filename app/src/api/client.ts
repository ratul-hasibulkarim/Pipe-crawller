// REST client for the Pi companion API (/api/v1). Every command returns the MCU's ACK status.
import type { AckStatusName, TelemetryMsg } from "../types/protocol";

export type CommandStatus = AckStatusName | "NO_LINK" | "TIMEOUT" | "RATE_LIMITED" | "SENT" | "NETWORK";

export interface CommandResult {
  status: CommandStatus;
  seq?: number;
}

/** Telemetry as served by the Pi: protocol message + freshness fields. */
export type TelemetrySnapshot = Partial<TelemetryMsg> & {
  age_ms: number | null;
  stale: boolean;
  link: boolean;
  rtt_ms: number | null;
};

export interface RobotConfig {
  size: string;
  pipe_id_min_mm: number;
  pipe_id_max_mm: number;
  max_speed_mm_s: number;
  max_grip_n: number;
  working_grip_n: number;
  default_grip_n: number;
  grip_low_n: number;
  telemetry_hz: number;
  drive_ttl_ms: number;
  tether_length_m: number;
  battery_series: number;
  gimbal_tilt_range: [number, number];
  params: string[];
}

export interface Health {
  status: "ok" | "degraded";
  version: string;
  size: string;
  link: boolean;
  telemetry_age_ms: number | null;
  rtt_ms: number | null;
  video_frames: number;
  recording: string | null;
  tether_m: number;
  protocol_version: number;
}

export interface PreflightCheck {
  name: string;
  ok: boolean;
  detail: string;
}

export interface RunSummary {
  id: string;
  started: number;
  ended: number | null;
  operator: string;
  distance_m: number | null;
  frames: number;
  events: number;
}

export interface RunPoint {
  t: number;
  distance_m: string;
  pipe_orientation_deg: string;
  grip0: string;
  grip1: string;
  grip2: string;
  cur0: string;
  cur3: string;
  est_diameter_mm: string;
  linear_mm_s: string;
  state: string;
}

export interface RunEvent {
  type: string;
  label?: string;
  t_wall: number;
  code?: string;
  file?: string;
  distance_m?: number;
}

const BASE = "/api/v1";

async function json<T>(path: string, init?: RequestInit): Promise<T> {
  const r = await fetch(BASE + path, { headers: { "content-type": "application/json" }, ...init });
  if (!r.ok && r.status !== 409 && r.status !== 503 && r.status !== 429 && r.status !== 422) {
    throw new Error(`${r.status} ${path}`);
  }
  return (await r.json()) as T;
}

async function command(path: string, body: object = {}): Promise<CommandResult> {
  try {
    return await json<CommandResult>(path, { method: "POST", body: JSON.stringify(body) });
  } catch {
    return { status: "NETWORK" };
  }
}

export const api = {
  health: () => json<Health>("/health"),
  config: () => json<RobotConfig>("/config"),
  preflight: () => json<{ ok: boolean; checks: PreflightCheck[] }>("/preflight"),
  drive: (linear_mm_s: number, turn_deg_s = 0, bend_dir_deg = 0) => command("/drive", { linear_mm_s, turn_deg_s, bend_dir_deg }),
  stop: () => command("/stop"),
  expand: (mode: "DIAMETER" | "FORCE" | "RETRACT", target?: number) => command("/expand", { mode, target }),
  cancelExpand: () => fetch(BASE + "/expand", { method: "DELETE" }),
  gimbal: (pan_deg: number, tilt_deg: number) => command("/gimbal", { pan_deg, tilt_deg }),
  light: (white_pct: number, ir_pct: number) => command("/light", { white_pct, ir_pct }),
  estop: () => command("/estop"),
  clearFault: () => command("/clear_fault"),
  setParam: (name: string, value: number) => command("/param", { name, value }),
  startRecording: (operator: string, pipe_id_range: string) => json<{ id: string }>("/recording/start", { method: "POST", body: JSON.stringify({ operator, pipe_id_range }) }),
  stopRecording: () => json<{ id: string | null }>("/recording/stop", { method: "POST" }),
  mark: (label: string) => json<{ ok: boolean }>("/mark", { method: "POST", body: JSON.stringify({ label }) }),
  snapshot: () => json<{ file: string | null; recording: boolean }>("/snapshot", { method: "POST" }),
  recordings: () => json<RunSummary[]>("/recordings"),
  recording: (id: string) => json<{ meta: Record<string, unknown>; events: RunEvent[]; snapshots: string[] }>(`/recordings/${id}`),
  recordingSeries: (id: string) => json<RunPoint[]>(`/recordings/${id}/telemetry.json?step=2`),
  videoUrl: (id: string) => `${BASE}/recordings/${id}/video.mp4`,
  csvUrl: (id: string, start?: number, end?: number) =>
    `${BASE}/recordings/${id}/export.csv` + (start !== undefined && end !== undefined ? `?start_s=${start}&end_s=${end}` : ""),
  clipUrl: (id: string, start: number, end: number) => `${BASE}/recordings/${id}/clip.mp4?start_s=${start}&end_s=${end}`,
  snapshotUrl: (id: string, name: string) => `${BASE}/recordings/${id}/snapshots/${name}`,
};

export function isOk(r: CommandResult): boolean {
  return r.status === "OK" || r.status === "ESTOPPED" || r.status === "SENT";
}
