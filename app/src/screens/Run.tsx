import { useEffect, useState } from "react";

import { api } from "../api/client";
import { StripChart } from "../components/StripChart";
import { useRobot } from "../state/robotStore";

export function Run(): JSX.Element {
  const history = useRobot((s) => s.history);
  const markers = useRobot((s) => s.markers);
  const recId = useRobot((s) => s.recordingId);
  const [snaps, setSnaps] = useState<string[]>([]);
  useEffect(() => {
    if (!recId) return;
    const load = () => api.recording(recId).then((r) => setSnaps(r.snapshots)).catch(() => undefined);
    void load();
    const id = setInterval(load, 3000);
    return () => clearInterval(id);
  }, [recId]);
  const h = history.slice(-600);
  const t = h.map((p) => p.t);
  const t0 = t[0] ?? 0;
  const rel = t.map((v) => v - t0);
  const m = markers.filter((x) => x.t >= t0).map((x) => ({ ...x, t: x.t - t0 }));
  return (
    <div className="p-3 space-y-3 overflow-auto h-full">
      <h2 className="font-semibold">Run (last {rel.length ? Math.round(rel[rel.length - 1]) : 0} s)</h2>
      <StripChart t={rel} markers={m} unit=" m" minSpan={1} series={[{ name: "distance m", color: "#3ddc84", values: h.map((p) => p.distance) }]} />
      <StripChart t={rel} markers={m} unit="°" minSpan={20} series={[{ name: "pipe orientation °", color: "#7cc7ff", values: h.map((p) => p.orientation) }]} />
      <StripChart t={rel} markers={m} minSpan={20} series={[{ name: "min grip N", color: "#ffb020", values: h.map((p) => p.grip) }, { name: "motor current A ×10", color: "#ff6b6b", values: h.map((p) => p.current * 10) }]} />
      <h3 className="font-semibold">Snapshots {recId ? "" : "(start recording to capture)"}</h3>
      <div className="flex gap-2 overflow-x-auto pb-2">
        {recId && snaps.map((s) => (
          <figure key={s} className="shrink-0">
            <img src={api.snapshotUrl(recId, s)} alt={s} className="h-28 rounded" />
            <figcaption className="text-[10px] text-hud-dim">{new Date(Number(s.replace(/\D/g, ""))).toLocaleTimeString()}</figcaption>
          </figure>
        ))}
      </div>
    </div>
  );
}
