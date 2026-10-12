import { useState } from "react";

import { DriveJoystick } from "../components/DriveJoystick";
import { EStopButton } from "../components/EStopButton";
import { ExpansionControl } from "../components/ExpansionControl";
import { GimbalPad } from "../components/GimbalPad";
import { LightControl } from "../components/LightControl";
import { TelemetryHUD } from "../components/TelemetryHUD";
import { VideoCanvas } from "../components/VideoCanvas";
import { useRecording } from "../hooks/useRecording";
import { useStale } from "../hooks/useTelemetry";
import { useRobot } from "../state/robotStore";

export function Live(): JSX.Element {
  const stale = useStale();
  const state = useRobot((s) => s.telemetry?.state);
  const hud = useRobot((s) => s.settings.hud);
  const setSettings = useRobot((s) => s.setSettings);
  const rec = useRecording();
  const [panel, setPanel] = useState<"arms" | "camera">("arms");
  const canDrive = !stale && (state === "GRIPPED" || state === "HOLDING" || state === "CRAWLING");
  const btn = "min-h-touch min-w-touch rounded-lg px-3 text-sm font-semibold";
  return (
    <div className="relative h-full w-full">
      <div className={stale ? "stale-ui absolute inset-0" : "absolute inset-0"}>
        <VideoCanvas />
        <TelemetryHUD />
      </div>
      <div className="absolute top-2 right-2 flex gap-1 z-20">
        <button className={`${btn} ${rec.recording ? "bg-red-600 animate-pulse" : "bg-black/50"}`} onClick={rec.toggle} disabled={rec.busy} aria-pressed={rec.recording}>
          {rec.recording ? "● REC" : "REC"}
        </button>
        <button className={`${btn} bg-black/50`} onClick={() => void rec.snapshot()} aria-label="Snapshot">📷</button>
        <button className={`${btn} bg-black/50`} onClick={() => void rec.mark("defect")} aria-label="Mark event">⚑</button>
        <button className={`${btn} bg-black/50`} onClick={() => setSettings({ hud: !hud })} aria-pressed={hud}>HUD</button>
      </div>
      {/* control deck: bottom third, thumb reachable */}
      <div className="absolute bottom-0 left-0 right-0 z-20 bg-gradient-to-t from-black/80 to-transparent pt-6 pb-3 px-3 flex items-end justify-between gap-2">
        <DriveJoystick disabled={!canDrive} />
        <div className="flex flex-col items-center gap-2">
          <div className="flex gap-1">
            <button className={`${btn} ${panel === "arms" ? "bg-white/30" : "bg-white/10"}`} onClick={() => setPanel("arms")}>Arms</button>
            <button className={`${btn} ${panel === "camera" ? "bg-white/30" : "bg-white/10"}`} onClick={() => setPanel("camera")}>Camera</button>
          </div>
          {panel === "arms" ? <ExpansionControl disabled={stale} /> : (<div className="flex gap-2 items-end"><GimbalPad /><LightControl /></div>)}
        </div>
      </div>
      {/* E-STOP floats above the deck so it is never squeezed off a narrow phone screen */}
      <div className="absolute right-3 bottom-64 z-30"><EStopButton /></div>
    </div>
  );
}
