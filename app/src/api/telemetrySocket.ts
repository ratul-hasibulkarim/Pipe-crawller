// Telemetry WebSocket with automatic reconnect. The UI never trusts it blindly: the store
// computes staleness from the receive time as well as the server's own `stale` flag.
import type { TelemetrySnapshot } from "./client";

export type TelemetryHandler = (t: TelemetrySnapshot) => void;

export function wsUrl(path: string): string {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${window.location.host}${path}`;
}

export class TelemetrySocket {
  private ws: WebSocket | null = null;
  private stopped = false;
  private retry = 250;

  constructor(
    private readonly onTelemetry: TelemetryHandler,
    private readonly onState: (open: boolean) => void,
    private readonly url = wsUrl("/api/v1/telemetry"),
  ) {}

  start(): void {
    this.stopped = false;
    this.open();
  }

  stop(): void {
    this.stopped = true;
    this.ws?.close();
  }

  private open(): void {
    const ws = new WebSocket(this.url);
    this.ws = ws;
    ws.onopen = () => {
      this.retry = 250;
      this.onState(true);
    };
    ws.onmessage = (ev: MessageEvent<string>) => {
      try {
        this.onTelemetry(JSON.parse(ev.data) as TelemetrySnapshot);
      } catch {
        /* ignore malformed frames */
      }
    };
    ws.onclose = () => {
      this.onState(false);
      if (!this.stopped) {
        setTimeout(() => this.open(), this.retry);
        this.retry = Math.min(this.retry * 2, 4000);
      }
    };
  }
}

/** Control channel: drive at ~10 Hz while the stick is held (dead-man). */
export class ControlSocket {
  private ws: WebSocket | null = null;
  private id = 0;

  open(): void {
    if (this.ws && this.ws.readyState <= 1) return;
    this.ws = new WebSocket(wsUrl("/api/v1/control"));
  }

  get ready(): boolean {
    return this.ws?.readyState === WebSocket.OPEN;
  }

  send(msg: Record<string, unknown>): boolean {
    if (!this.ready || !this.ws) return false;
    this.ws.send(JSON.stringify({ id: ++this.id, ...msg }));
    return true;
  }

  close(): void {
    this.ws?.close();
  }
}
