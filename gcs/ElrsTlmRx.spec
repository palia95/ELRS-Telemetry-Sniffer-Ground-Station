# -*- mode: python ; coding: utf-8 -*-
# PyInstaller spec for the ELRS TLM RX native macOS app.
#
#   pyinstaller ElrsTlmRx.spec --noconfirm      ->  dist/ELRS TLM RX.app
#
# For a universal2 (arm64 + x86_64) binary set TARGET_ARCH = 'universal2' below,
# but note that requires a universal2 Python AND universal2 wheels for every
# native dependency (aiohttp, pyobjc). The default (None) builds for the host
# architecture, which is what you want for a personal Apple-Silicon Mac.
from PyInstaller.utils.hooks import collect_all, collect_submodules

TARGET_ARCH = None   # or 'universal2'

datas, binaries, hiddenimports = [('static', 'static')], [], []

# Pull in everything these packages need (backends, data files, dynamic imports).
for pkg in ('bleak', 'webview', 'aiohttp', 'serial'):
    d, b, h = collect_all(pkg)
    datas += d
    binaries += b
    hiddenimports += h

# pyobjc frameworks used by pywebview (WebKit/Cocoa) and bleak (CoreBluetooth).
hiddenimports += collect_submodules('objc')
hiddenimports += ['Foundation', 'Cocoa', 'WebKit', 'CoreBluetooth', 'Quartz', 'Security']

a = Analysis(
    ['app.py'],
    pathex=[],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz, a.scripts, [],
    exclude_binaries=True,
    name='ElrsTlmRx',
    debug=False,
    strip=False,
    upx=False,
    console=False,
    target_arch=TARGET_ARCH,
)
coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False, name='ElrsTlmRx')

app = BUNDLE(
    coll,
    name='ELRS TLM RX.app',
    icon=None,                          # drop an .icns path here for a custom icon
    bundle_identifier='com.elrstlmrx.app',
    info_plist={
        'CFBundleName': 'ELRS TLM RX',
        'CFBundleDisplayName': 'ELRS TLM RX',
        'CFBundleShortVersionString': '0.1.0',
        'LSMinimumSystemVersion': '11.0',
        'NSHighResolutionCapable': True,
        # REQUIRED or macOS silently blocks BLE for a bundled app:
        'NSBluetoothAlwaysUsageDescription':
            'ELRS TLM RX uses Bluetooth to receive drone telemetry from the sniffer.',
        'NSBluetoothPeripheralUsageDescription':
            'ELRS TLM RX uses Bluetooth to receive drone telemetry from the sniffer.',
    },
)
