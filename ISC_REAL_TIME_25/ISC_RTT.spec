# -*- mode: python ; coding: utf-8 -*-


excludes = [
    'PyQt5.QtWebEngine', 'PyQt5.QtWebEngineCore', 'PyQt5.QtWebEngineWidgets',
    'PyQt5.QtQml', 'PyQt5.QtQuick', 'PyQt5.Qt3D', 'PyQt5.QtMultimedia',
    'PyQt5.QtSql', 'PyQt5.QtDesigner', 'PyQt5.QtBluetooth', 'PyQt5.QtSensors',
    'PyQt5.QtPositioning', 'PyQt5.QtXml', 'PyQt5.QtNfc',
    'tkinter', 'tcl', 'scipy', 'curses',
    'matplotlib.tests', 'matplotlib.testing', 'numpy.tests'
]

a = Analysis(
    ['ui.py'],
    pathex=[],
    binaries=[],
    datas=[('isc_logo.png', '.'), ('isc_logo.ico', '.')],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=excludes,
    noarchive=False,
    optimize=2,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='ISC_RTT',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=['isc_logo.ico'],
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='ISC_RTT',
)
