"""Generated Python protocol bindings."""

from pi.app import protocol_gen as P


def test_crc_known_vector() -> None:
    """CRC16-CCITT-FALSE check value."""
    assert P.crc16(b"123456789") == 0x29B1


def test_round_trip_and_resync() -> None:
    """Garbage before a frame and a frame split across reads."""
    d = P.DriveMsg(seq=9, linear_mm_s=-120, turn_deg_s=5, bend_dir_deg=270, ttl_ms=200)
    dec = P.FrameDecoder()
    out = dec.feed(b"\x00\xaa\x13" + P.encode(d)[:4]) + dec.feed(P.encode(d)[4:])
    assert out == [d]


def test_bad_crc_counted() -> None:
    """One flipped bit -> dropped and counted."""
    f = bytearray(P.encode(P.LightMsg(seq=1, white_pct=50)))
    f[6] ^= 1
    dec = P.FrameDecoder()
    assert dec.feed(bytes(f)) == []
    assert dec.crc_errors == 1


def test_telemetry_json_shape_matches_spec() -> None:
    """JSON telemetry has the nested shape from the spec."""
    t = P.TelemetryMsg(state=int(P.State.CRAWLING), faults=(1 << P.Fault.SLIP_WARNING), grip_force_n=[31.2, 30.8, 31.5], battery_v=14.9)
    j = t.to_json()
    assert j["state"] == "CRAWLING"
    assert j["faults"] == ["SLIP_WARNING"]
    assert j["imu"].keys() >= {"pitch", "roll", "yaw"}
    assert j["battery"]["v"] == 14.9
    assert len(j["grip_force_n"]) == 3
