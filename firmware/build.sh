#!/usr/bin/env bash
# =====================================================================
# One-command build of the T3-S3 Ghost sniffer firmware.
#
#   ./build.sh /path/to/ExpressLRS   [env]
#
# - Verifies the ELRS tree is 3.5.x
# - Runs integrate.py (copies sniffer sources, patches rx_main.cpp)
# - Installs the PlatformIO target env
# - Builds; prints the firmware.bin path
#
# Requires PlatformIO (`pip install platformio` or the VS Code extension).
# Default env is the serial-debug build.
# =====================================================================
set -euo pipefail

ELRS="${1:?usage: ./build.sh /path/to/ExpressLRS [env] [binding_phrase]}"
ENV="${2:-T3S3_Sniffer_2400_RX_Serial}"
PHRASE="${3:-}"        # target link's binding phrase (optional if already set)
HERE="$(cd "$(dirname "$0")" && pwd)"

SRC="$ELRS/src"
[ -f "$SRC/src/rx_main.cpp" ] || { echo "ERROR: $SRC/src/rx_main.cpp not found"; exit 1; }

# 0. sanity: version
if [ -f "$SRC/VERSION" ]; then
  echo "ELRS VERSION: $(head -1 "$SRC/VERSION")"
fi

# 1. integrate sniffer + patch rx_main (idempotent)
python3 "$HERE/integrate.py" "$SRC"

# 1b. set regulatory domain + binding phrase in user_defines.txt (ELRS-native).
#     build_flags.py converts MY_BINDING_PHRASE -> -DMY_UID at build time.
python3 - "$SRC/user_defines.txt" "$PHRASE" <<'PY'
import sys, os, re
path, phrase = sys.argv[1], sys.argv[2]
s = open(path).read() if os.path.exists(path) else ""

# ensure ISM 2400 domain is present & uncommented
if re.search(r'(?m)^\s*-DRegulatory_Domain_ISM_2400\s*$', s):
    pass
elif re.search(r'(?m)^\s*#\s*-DRegulatory_Domain_ISM_2400\s*$', s):
    s = re.sub(r'(?m)^\s*#\s*-DRegulatory_Domain_ISM_2400\s*$', '-DRegulatory_Domain_ISM_2400', s)
else:
    s += ("\n" if s and not s.endswith("\n") else "") + "-DRegulatory_Domain_ISM_2400\n"

# set/replace the binding phrase if one was provided
if phrase:
    s = re.sub(r'(?m)^\s*#?\s*-DMY_BINDING_PHRASE=.*$', '', s)   # drop any existing
    s = s.rstrip() + '\n-DMY_BINDING_PHRASE="%s"\n' % phrase
    print('user_defines.txt: set binding phrase "%s" + ISM2400 domain' % phrase)
else:
    has = re.search(r'(?m)^\s*-DMY_BINDING_PHRASE=.*$', s)
    print('user_defines.txt: ISM2400 domain ensured; '
          + ('phrase kept: ' + has.group(0).strip() if has
             else 'NO PHRASE SET -> pass it as the 3rd arg, or the sniffer cannot lock'))

open(path, "w").write(s)
PY

# 2. install target env + register it in platformio.ini extra_configs
cp "$HERE/targets/t3s3_sniffer.ini" "$SRC/targets/t3s3_sniffer.ini"
echo "installed targets/t3s3_sniffer.ini"

# 2a. stage the hardware layout into the SPIFFS filesystem image.
#     hardware_init() reads /hardware.json from SPIFFS first, so this makes the
#     board boot CONFIGURED (no WiFi upload, avoids the unconfigured crash path).
mkdir -p "$SRC/data"
cp "$HERE/hardware/lilygo_t3s3_sniffer_rx.min.json" "$SRC/data/hardware.json"
echo "staged data/hardware.json"

# 2b. UNIFIED firmware loads the UID at RUNTIME from options.json (not the
#     compile-time MY_UID). Write data/options.json with the UID derived from
#     the SAME phrase, so the sniffer locks to the target link.
python3 - "$SRC/user_defines.txt" "$SRC/data/options.json" "$PHRASE" <<'PY'
import sys, os, re, hashlib, json
ud, outp, phrase = sys.argv[1], sys.argv[2], sys.argv[3]
if not phrase:
    s = open(ud).read() if os.path.exists(ud) else ""
    m = re.search(r'(?m)^\s*-DMY_BINDING_PHRASE="(.*)"\s*$', s)
    phrase = m.group(1) if m else ""
if not phrase:
    print("options.json: NO PHRASE -> UID stays zero (cannot lock). Pass phrase as 3rd arg.")
else:
    import struct
    digest = hashlib.md5(('-DMY_BINDING_PHRASE="%s"' % phrase).encode()).digest()
    uid = list(digest[0:6])
    # A NON-ZERO flash-discriminator is required: ELRS only copies this UID into
    # the stored config when the discriminator differs from what's saved. Derive
    # it from the phrase so it's stable per-phrase but non-zero (0 == "unset").
    disc = struct.unpack('<I', digest[6:10])[0] or 0xE1725A9D
    json.dump({"uid": uid, "flash-discriminator": disc}, open(outp, "w"))
    print("options.json: uid=%s flash-discriminator=%d (phrase '%s')" % (uid, disc, phrase))
PY
echo "staged data/ -> flash with: pio run -e $ENV -t uploadfs"
python3 - "$SRC/platformio.ini" <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
if "targets/t3s3_sniffer.ini" in s:
    print("platformio.ini: already registers t3s3_sniffer.ini")
else:
    anchor = "\ttargets/common.ini\n"
    if anchor not in s:
        print("platformio.ini: could not find extra_configs anchor 'targets/common.ini'"); sys.exit(3)
    s = s.replace(anchor, anchor + "\ttargets/t3s3_sniffer.ini\n", 1)
    open(p, "w").write(s)
    print("platformio.ini: registered targets/t3s3_sniffer.ini")
PY

# 3. build
cd "$SRC"
# Resolve the PlatformIO launcher: PATH `pio`, ~/.local/bin/pio, or python -m platformio
if command -v pio >/dev/null 2>&1; then
  PIO="pio"
elif [ -x "$HOME/.local/bin/pio" ]; then
  PIO="$HOME/.local/bin/pio"
elif python3 -c "import platformio" >/dev/null 2>&1; then
  PIO="python3 -m platformio"
else
  echo "PlatformIO not found. Install with: pip install platformio"; exit 2
fi
echo "Using PlatformIO: $PIO"
$PIO run -e "$ENV"

BIN="$SRC/.pio/build/$ENV/firmware.bin"
echo
echo "==================================================================="
echo "Build OK. Firmware: $BIN"
echo "Flash it: see docs/02_build_flash_guide.md (esptool / PlatformIO)."
echo "Remember to also flash hardware/lilygo_t3s3_sniffer_rx.json via the"
echo "ELRS WiFi web UI after the first USB flash."
echo "==================================================================="
