import { useCallback, useState } from "react";

import { api } from "../api/client";
import { useRobot } from "../state/robotStore";

export function useRecording(): {
  recording: boolean;
  busy: boolean;
  toggle: () => Promise<void>;
  snapshot: () => Promise<string | null>;
  mark: (label: string) => Promise<void>;
} {
  const id = useRobot((s) => s.recordingId);
  const setRecording = useRobot((s) => s.setRecording);
  const addMarker = useRobot((s) => s.addMarker);
  const operator = useRobot((s) => s.settings.operator);
  const cfg = useRobot((s) => s.config);
  const [busy, setBusy] = useState(false);
  const toggle = useCallback(async () => {
    setBusy(true);
    try {
      if (id) {
        await api.stopRecording();
        setRecording(null);
      } else {
        const range = cfg ? `${cfg.pipe_id_min_mm}-${cfg.pipe_id_max_mm}` : "";
        setRecording((await api.startRecording(operator, range)).id);
      }
    } finally {
      setBusy(false);
    }
  }, [id, setRecording, operator, cfg]);
  const snapshot = useCallback(async () => {
    const r = await api.snapshot();
    addMarker("snapshot");
    return r.file;
  }, [addMarker]);
  const mark = useCallback(
    async (label: string) => {
      await api.mark(label);
      addMarker(label);
    },
    [addMarker],
  );
  return { recording: !!id, busy, toggle, snapshot, mark };
}
