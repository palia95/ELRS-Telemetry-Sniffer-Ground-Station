#!/usr/bin/env python3
"""
ELRS TLM RX - native macOS app.

Runs the same aiohttp backend + dashboard as gcs.py, but hosts the UI in a native
WKWebView window (via pywebview) instead of a browser tab. Serial/BLE transports,
CRSF decoding, CSV logging and the map/attitude dashboard are all reused unchanged.

  python3 app.py            # run from source
  ./build_app.sh            # package into "ELRS TLM RX.app"

Architecture:
  * The asyncio backend (aiohttp server + bleak + serial thread) runs in a
    background thread with its own event loop.
  * pywebview owns the macOS main thread (required by Cocoa) and shows the window
    pointing at the local server once it's listening.
"""
import asyncio
import socket
import sys
import threading
import time
import types

import webview

import gcs

HOST = "127.0.0.1"
PORT = 8760


def _run_backend():
    """Run the aiohttp/asyncio backend forever in this thread's own loop."""
    ns = types.SimpleNamespace(
        serial=None,      # auto-pick a serial cable if present
        ble=None,         # else idle; user scans/connects from the UI
        csv=False,
        baud=460800,
        http_port=PORT,
        host=HOST,
    )
    try:
        asyncio.run(gcs.main_async(ns))
    except Exception as e:      # keep the window up even if the backend dies
        print(f"[app] backend stopped: {e}", file=sys.stderr)


def _wait_for_server(host, port, timeout=15.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.4):
                return True
        except OSError:
            time.sleep(0.2)
    return False


def main():
    threading.Thread(target=_run_backend, daemon=True).start()
    if not _wait_for_server(HOST, PORT):
        print("[app] backend did not start; opening window anyway", file=sys.stderr)

    webview.create_window(
        "ELRS TLM RX",
        f"http://{HOST}:{PORT}/",
        width=1240, height=820, min_size=(960, 640),
    )
    # gui=None lets pywebview pick the Cocoa/WKWebView backend on macOS.
    webview.start()


if __name__ == "__main__":
    main()
