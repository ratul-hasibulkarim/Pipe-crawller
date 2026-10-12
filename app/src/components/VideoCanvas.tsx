// Live video: WebRTC first, MJPEG fallback. Drag = pan/tilt, double-tap = re-centre,
// pinch / wheel = digital zoom. All gestures are on the video itself.
import { useEffect, useRef, useState } from "react";

import { api } from "../api/client";
import { connectWebRtc } from "../api/webrtc";
import { useCommand } from "../hooks/useCommand";
import { useRobot } from "../state/robotStore";

const DEG_PER_PX = 0.25;

export function VideoCanvas(): JSX.Element {
  const videoRef = useRef<HTMLVideoElement>(null);
  const [mode, setMode] = useState<"connecting" | "webrtc" | "mjpeg">("connecting");
  const [zoom, setZoom] = useState(1);
  const tel = useRobot((s) => s.telemetry);
  const pan = useCommand("pan");
  const tilt = useCommand("tilt");
  const target = useRef({ pan: 0, tilt: 0 });
  const drag = useRef<{ x: number; y: number; pan: number; tilt: number } | null>(null);
  const pinch = useRef<{ d: number; z: number } | null>(null);
  const pointers = useRef(new Map<number, { x: number; y: number }>());
  const lastSend = useRef(0);
  const lastTap = useRef(0);

  useEffect(() => {
    let pc: RTCPeerConnection | null = null;
    let alive = true;
    connectWebRtc().then((r) => {
      if (!alive) return r?.pc.close();
      if (r && videoRef.current) {
        pc = r.pc;
        videoRef.current.srcObject = r.stream;
        setMode("webrtc");
      } else setMode("mjpeg");
    });
    return () => {
      alive = false;
      pc?.close();
    };
  }, []);

  const sendGimbal = (p: number, t: number, force = false): void => {
    const now = Date.now();
    target.current = { pan: p, tilt: t };
    if (!force && now - lastSend.current < 150) return;
    lastSend.current = now;
    void pan.send(p, () => api.gimbal(p, t));
    useRobot.getState().setPending("tilt", t); // confirmed by telemetry like pan
  };

  const onDown = (e: React.PointerEvent): void => {
    (e.target as Element).setPointerCapture(e.pointerId);
    pointers.current.set(e.pointerId, { x: e.clientX, y: e.clientY });
    if (pointers.current.size === 2) {
      const [a, b] = [...pointers.current.values()];
      pinch.current = { d: Math.hypot(a.x - b.x, a.y - b.y), z: zoom };
      drag.current = null;
      return;
    }
    const now = Date.now();
    if (now - lastTap.current < 300) {
      sendGimbal(0, 0, true);
      setZoom(1);
    }
    lastTap.current = now;
    const g = tel?.gimbal;
    drag.current = { x: e.clientX, y: e.clientY, pan: g?.pan_deg ?? target.current.pan, tilt: g?.tilt_deg ?? target.current.tilt };
  };
  const onMove = (e: React.PointerEvent): void => {
    if (!pointers.current.has(e.pointerId)) return;
    pointers.current.set(e.pointerId, { x: e.clientX, y: e.clientY });
    if (pinch.current && pointers.current.size === 2) {
      const [a, b] = [...pointers.current.values()];
      setZoom(Math.max(1, Math.min(4, (pinch.current.z * Math.hypot(a.x - b.x, a.y - b.y)) / pinch.current.d)));
      return;
    }
    if (!drag.current) return;
    const p = drag.current.pan - (e.clientX - drag.current.x) * DEG_PER_PX;
    const t = Math.max(-90, Math.min(90, drag.current.tilt + (e.clientY - drag.current.y) * DEG_PER_PX));
    sendGimbal(((p + 540) % 360) - 180, t);
  };
  const onUp = (e: React.PointerEvent): void => {
    pointers.current.delete(e.pointerId);
    if (pointers.current.size < 2) pinch.current = null;
    if (drag.current) sendGimbal(target.current.pan, target.current.tilt, true);
    drag.current = null;
  };

  return (
    <div
      className="absolute inset-0 overflow-hidden bg-black touch-none select-none"
      onPointerDown={onDown}
      onPointerMove={onMove}
      onPointerUp={onUp}
      onPointerCancel={onUp}
      onWheel={(e) => setZoom((z) => Math.max(1, Math.min(4, z - e.deltaY * 0.002)))}
      aria-label="Camera view: drag to pan and tilt, double-tap to centre, pinch to zoom"
    >
      <div className="w-full h-full flex items-center justify-center" style={{ transform: `scale(${zoom})` }}>
        {mode === "mjpeg" ? (
          <img src="/stream.mjpeg" alt="camera" className="w-full h-full object-contain" draggable={false} />
        ) : (
          <video ref={videoRef} autoPlay playsInline muted className="w-full h-full object-contain" />
        )}
      </div>
      <div className="absolute bottom-1 right-2 text-[10px] text-hud-dim hud-text pointer-events-none">
        {mode === "connecting" ? "connecting video..." : mode.toUpperCase()} {zoom > 1 ? `x${zoom.toFixed(1)}` : ""}
        {(pan.pending || tilt.pending) && " · aiming..."}
      </div>
    </div>
  );
}
