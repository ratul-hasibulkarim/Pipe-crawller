import { useState } from "react";

import { ConnectionBadge } from "./components/ConnectionBadge";
import { FaultBanner } from "./components/FaultBanner";
import { useStale, useTelemetryConnection } from "./hooks/useTelemetry";
import { Connect } from "./screens/Connect";
import { Live } from "./screens/Live";
import { Recordings } from "./screens/Recordings";
import { Run } from "./screens/Run";
import { Settings } from "./screens/Settings";

type Tab = "connect" | "live" | "run" | "recordings" | "settings";
const TABS: { id: Tab; label: string }[] = [
  { id: "connect", label: "Connect" },
  { id: "live", label: "Live" },
  { id: "run", label: "Run" },
  { id: "recordings", label: "Recordings" },
  { id: "settings", label: "Settings" },
];

export default function App(): JSX.Element {
  useTelemetryConnection();
  const stale = useStale();
  const [tab, setTab] = useState<Tab>("connect");
  return (
    <div className="h-full flex flex-col">
      <main className={`relative flex-1 min-h-0 ${tab === "live" ? "" : "pt-10"}`}>
        <FaultBanner />
        {stale && tab === "live" && (
          <div role="status" className="absolute top-14 left-2 right-2 z-20 rounded bg-red-800/90 text-center py-2 font-semibold">
            ROBOT LINK STALE - values frozen, controls disabled
          </div>
        )}
        <div className="absolute top-2 left-2 z-20"><ConnectionBadge /></div>
        {tab === "connect" && <Connect onReady={() => setTab("live")} />}
        {tab === "live" && <Live />}
        {tab === "run" && <Run />}
        {tab === "recordings" && <Recordings />}
        {tab === "settings" && <Settings />}
      </main>
      <nav className="flex bg-black/80 border-t border-white/10" style={{ paddingBottom: "env(safe-area-inset-bottom)" }}>
        {TABS.map((t) => (
          <button key={t.id} className={`flex-1 min-h-touch text-sm ${tab === t.id ? "text-white font-semibold" : "text-hud-dim"}`}
            aria-current={tab === t.id ? "page" : undefined} onClick={() => setTab(t.id)}>
            {t.label}
          </button>
        ))}
      </nav>
    </div>
  );
}
