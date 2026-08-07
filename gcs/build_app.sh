#!/usr/bin/env bash
# Build the native "ELRS TLM RX.app" bundle with PyInstaller.
set -euo pipefail
cd "$(dirname "$0")"

echo "== installing deps (runtime + pyinstaller) =="
python3 -m pip install -r requirements.txt pyinstaller

echo "== building app =="
rm -rf build dist
python3 -m PyInstaller ElrsTlmRx.spec --noconfirm

echo
echo "Built: $(pwd)/dist/ELRS TLM RX.app"
echo "Run it:  open 'dist/ELRS TLM RX.app'"
echo
echo "First launch: right-click -> Open (unsigned app), and approve the Bluetooth"
echo "permission prompt so BLE scanning works."
open dist 2>/dev/null || true
