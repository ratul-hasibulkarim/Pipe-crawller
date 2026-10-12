import { useState } from "react";

import { api } from "../api/client";
import { useCommand } from "../hooks/useCommand";
import { useRobot } from "../state/robotStore";

export function LightControl(): JSX.Element {
  const light = useRobot((s) => s.telemetry?.light);
  const [white, setWhite] = useState(0);
  const [ir, setIr] = useState(0);
  const w = useCommand("white");
  const i = useCommand("ir");
  const commit = (nw: number, ni: number): void => {
    void w.send(nw, () => api.light(nw, ni));
    useRobot.getState().setPending("ir", ni);
  };
  return (
    <div className="flex flex-col gap-1 w-40 text-xs hud-text">
      <label className="flex items-center gap-2">
        <span className="w-10">White</span>
        <input type="range" min={0} max={100} value={white} aria-label="White light %" className="flex-1 h-11 accent-yellow-300"
          onChange={(e) => setWhite(Number(e.target.value))} onPointerUp={() => commit(white, ir)} onKeyUp={() => commit(white, ir)} />
        <span className={`w-12 text-right ${w.pending ? "text-hud-warn" : ""}`}>{light?.white_pct ?? "--"}%</span>
      </label>
      <label className="flex items-center gap-2">
        <span className="w-10">IR</span>
        <input type="range" min={0} max={100} value={ir} aria-label="Infra-red light %" className="flex-1 h-11 accent-rose-400"
          onChange={(e) => setIr(Number(e.target.value))} onPointerUp={() => commit(white, ir)} onKeyUp={() => commit(white, ir)} />
        <span className={`w-12 text-right ${i.pending ? "text-hud-warn" : ""}`}>{light?.ir_pct ?? "--"}%</span>
      </label>
    </div>
  );
}
