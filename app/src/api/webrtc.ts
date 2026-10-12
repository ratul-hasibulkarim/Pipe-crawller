// WebRTC receive-only video from the Pi (aiortc). Returns null on failure so the caller
// can fall back to MJPEG.
export async function connectWebRtc(timeoutMs = 5000): Promise<{ stream: MediaStream; pc: RTCPeerConnection } | null> {
  if (typeof RTCPeerConnection === "undefined") return null;
  const pc = new RTCPeerConnection({ iceServers: [] }); // same LAN as the reel AP: no STUN needed
  pc.addTransceiver("video", { direction: "recvonly" });
  const stream = new MediaStream();
  pc.ontrack = (ev) => stream.addTrack(ev.track);
  try {
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    await new Promise<void>((resolve) => {
      if (pc.iceGatheringState === "complete") resolve();
      else {
        pc.onicegatheringstatechange = () => pc.iceGatheringState === "complete" && resolve();
        setTimeout(resolve, 1500);
      }
    });
    const ctl = new AbortController();
    const timer = setTimeout(() => ctl.abort(), timeoutMs);
    const r = await fetch("/api/v1/webrtc/offer", {
      method: "POST",
      headers: { "content-type": "application/json" },
      body: JSON.stringify({ sdp: pc.localDescription?.sdp, type: pc.localDescription?.type }),
      signal: ctl.signal,
    });
    clearTimeout(timer);
    if (!r.ok) throw new Error(String(r.status));
    const answer = (await r.json()) as RTCSessionDescriptionInit;
    await pc.setRemoteDescription(answer);
    return { stream, pc };
  } catch {
    pc.close();
    return null;
  }
}
