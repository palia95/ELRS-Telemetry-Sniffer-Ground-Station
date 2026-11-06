"""Tests for the GCS backend. Run:  python -m pytest test_gcs.py -q"""
import asyncio
import json
import os
import time

import aiohttp
import pytest
from aiohttp.test_utils import TestClient, TestServer

import crsf
import gcs


def run(coro):
    return asyncio.run(coro)


def make_frame(ftype, payload):
    body = bytes([ftype]) + bytes(payload)
    return bytes([0xC8, len(body) + 1]) + body + bytes([crsf.crc8_dvb_s2(body)])


def batt_frame():
    return make_frame(crsf.BATTERY_SENSOR, [0, 126, 0, 5, 0, 0, 10, 80])


@pytest.fixture(autouse=True)
def appdir(tmp_path, monkeypatch):
    monkeypatch.setattr(gcs, "_app_dir", lambda: str(tmp_path))
    return tmp_path


# ---- FrameStreamer ---------------------------------------------------------
def test_streamer_resync_bad_length():
    good1, good2 = batt_frame(), batt_frame()
    # plausible-but-bogus length (0x20) would swallow good1 without a CRC check
    junk = bytes([0xC8, 0x20, 0x01, 0x02])
    st = crsf.FrameStreamer()
    out = st.feed(junk + good1 + good2)
    assert out == [good1, good2]


def test_streamer_fragmented_and_corrupt():
    f = batt_frame()
    bad = bytearray(f)
    bad[-1] ^= 0xFF
    st = crsf.FrameStreamer()
    assert st.feed(f[:5]) == []
    assert st.feed(f[5:] + bytes(bad) + f) == [f, f]


def test_streamer_max_len():
    st = crsf.FrameStreamer(max_len=10)
    big = make_frame(crsf.FLIGHT_MODE, b"A" * 20)
    assert st.feed(big) == []


# ---- helpers ---------------------------------------------------------------
async def client_for(host="127.0.0.1"):
    hub = gcs.Hub(asyncio.get_running_loop(), 460800)
    client = TestClient(TestServer(gcs.build_app(hub, host)))
    await client.start_server()
    return hub, client


async def ws_status(client, headers):
    try:
        ws = await client.ws_connect("/ws", headers=headers)
    except aiohttp.WSServerHandshakeError as e:
        return e.status
    await ws.close()
    return 101


# ---- Origin / Host ---------------------------------------------------------
def test_ws_origin_host():
    async def go():
        hub, c = await client_for()
        port = c.port
        try:
            ok = f"127.0.0.1:{port}"
            assert await ws_status(c, {}) == 101                                  # no Origin
            assert await ws_status(c, {"Origin": f"http://{ok}"}) == 101          # app origin
            assert await ws_status(c, {"Origin": f"http://localhost:{port}",
                                       "Host": f"localhost:{port}"}) == 101
            assert await ws_status(c, {"Origin": "http://evil.example"}) == 403
            assert await ws_status(c, {"Origin": f"http://127.0.0.1:{port+1}"}) == 403
            assert await ws_status(c, {"Host": f"evil.example:{port}"}) == 403    # rebinding
            assert await ws_status(c, {"Host": f"evil.example:{port}",
                                       "Origin": f"http://evil.example:{port}"}) == 403
            # index page itself is not gated
            assert (await c.get("/")).status == 200
        finally:
            await c.close()
    run(go())


def test_ws_non_loopback_bind_accepts_any_host():
    async def go():
        hub, c = await client_for("0.0.0.0")
        try:
            h = f"gcs.lan:{c.port}"
            assert await ws_status(c, {"Host": h, "Origin": f"http://{h}"}) == 101
            assert await ws_status(c, {"Host": h, "Origin": "http://evil.example"}) == 403
        finally:
            await c.close()
    run(go())


def test_ws_ignores_non_object_json():
    async def go():
        hub, c = await client_for()
        try:
            ws = await c.ws_connect("/ws")
            await ws.receive(); await ws.receive()      # initial sources + telemetry
            for junk in ("[1,2]", "42", "null", '"x"', "{bad"):
                await ws.send_str(junk)
            await ws.send_str(json.dumps({"cmd": "list_serial"}))
            m = await asyncio.wait_for(ws.receive(), 3)      # still alive, still answering
            assert json.loads(m.data)["type"] == "sources" and not ws.closed
        finally:
            await c.close()
    run(go())


# ---- Tiles -----------------------------------------------------------------
PNG = b"\x89PNG fake"


def test_tiles(appdir, monkeypatch):
    calls = []

    async def fake_fetch(app, z, x, y):
        calls.append((z, x, y))
        return PNG if fake_fetch.ok else None
    fake_fetch.ok = True
    monkeypatch.setattr(gcs, "fetch_tile", fake_fetch)

    async def go():
        hub, c = await client_for()
        try:
            r = await c.get("/tiles/3/4/5.png")                      # miss -> fetch + cache
            assert r.status == 200 and await r.read() == PNG and len(calls) == 1
            path = appdir / "tiles" / "3" / "4" / "5.png"
            assert path.read_bytes() == PNG
            assert not [p for p in path.parent.iterdir() if p.suffix == ".tmp"]
            r = await c.get("/tiles/3/4/5.png")                      # hit
            assert r.status == 200 and len(calls) == 1

            old = time.time() - gcs.TILE_MAX_AGE - 100               # stale + fetch ok -> refreshed
            os.utime(path, (old, old))
            path.write_bytes(b"old")
            os.utime(path, (old, old))
            r = await c.get("/tiles/3/4/5.png")
            assert await r.read() == PNG and len(calls) == 2
            assert path.read_bytes() == PNG

            path.write_bytes(b"old")                                 # stale + fetch fails -> stale
            os.utime(path, (old, old))
            fake_fetch.ok = False
            r = await c.get("/tiles/3/4/5.png")
            assert r.status == 200 and await r.read() == b"old"

            assert (await c.get("/tiles/9/9/9.png")).status == 404   # miss + offline
            assert (await c.get("/tiles/a/1/1.png")).status == 404   # non-digit: no route
            assert (await c.get("/tiles/%D9%A1/1/1.png")).status == 404   # non-ASCII digit
        finally:
            await c.close()
    run(go())


def test_tile_atomic_write_cleans_up(appdir, monkeypatch):
    p = str(appdir / "t" / "a.png")

    def boom(a, b):
        raise OSError("nope")
    monkeypatch.setattr(os, "replace", boom)
    with pytest.raises(OSError):
        gcs.write_tile_atomic(p, PNG)
    assert not os.path.exists(p) and os.listdir(os.path.dirname(p)) == []


# ---- process_loop / logging ------------------------------------------------
def test_process_loop_survives_csv_oserror(appdir, monkeypatch):
    async def go():
        hub = gcs.Hub(asyncio.get_running_loop(), 460800)
        hub.logger.log_dir = str(appdir)
        hub.logger.start()
        assert hub.logger.active

        def boom(snap):
            raise OSError("disk full")
        monkeypatch.setattr(hub.logger, "write", boom)
        task = asyncio.create_task(hub.process_loop())
        hub.queue.put_nowait(batt_frame())
        hub.queue.put_nowait(b"\x00\x00\x00\x00")           # garbage frame
        hub.queue.put_nowait(batt_frame())
        await asyncio.sleep(0.2)
        assert not task.done()
        assert not hub.logger.active and "disk full" in hub.log_error
        assert hub.state.frames == 2                         # kept going
        assert json.loads(hub._sources_msg())["log_error"] == hub.log_error

        def explode(frame):
            raise RuntimeError("unexpected")
        monkeypatch.setattr(crsf, "parse_frame", explode)
        hub.queue.put_nowait(batt_frame())
        await asyncio.sleep(0.1)
        assert not task.done()
        task.cancel()
    run(go())


def test_ws_log_start_failure_keeps_socket(appdir, monkeypatch):
    async def go():
        hub, c = await client_for()
        try:
            hub.logger.log_dir = str(appdir)
            real_start = hub.logger.start

            def boom(path=None):
                raise OSError("read-only fs")
            monkeypatch.setattr(hub.logger, "start", boom)
            ws = await c.ws_connect("/ws")
            await ws.receive(); await ws.receive()
            await ws.send_str(json.dumps({"cmd": "log", "on": True}))
            m = json.loads((await asyncio.wait_for(ws.receive(), 3)).data)
            assert m["type"] == "sources" and "read-only fs" in m["log_error"] and not m["logging"]
            assert not ws.closed
            monkeypatch.setattr(hub.logger, "start", real_start)
            await ws.send_str(json.dumps({"cmd": "log", "on": True}))   # success clears the error
            m = json.loads((await asyncio.wait_for(ws.receive(), 3)).data)
            assert m["logging"] and m["log_error"] is None
            hub.logger.stop()
        finally:
            await c.close()
    run(go())
