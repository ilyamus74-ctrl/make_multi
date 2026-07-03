#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8') if Path('/root/new_yolo8').exists() else Path.cwd()
AUDIT = ROOT / 'ptz_contract_audit.py'

s = AUDIT.read_text(encoding='utf-8', errors='ignore')
backup = AUDIT.with_suffix(AUDIT.suffix + f'.bak_ui_detection_js_v1_{int(time.time())}')
backup.write_text(s, encoding='utf-8')

FUNC_NAME = 'audit_ui_detection_controls_contract'

new_func = r'''

def audit_ui_detection_controls_contract():
    section("LAYER 1B — UI DETECTION CONTROLS JS CONTRACT")

    from pathlib import Path

    web_path = Path("web/index.html")
    if not web_path.exists():
        add("UI detection controls JS web/index.html exists", False, "missing web/index.html")
        return

    web = web_path.read_text(encoding="utf-8", errors="ignore")

    required_exact_once = [
        "PTZ_CLEAN_DETECTION_UI_HYDRATE_F5_START",
        "PTZ_CLEAN_DETECTION_ACTIVE_PRESET_PERSIST_START",
    ]

    for token in required_exact_once:
        cnt = web.count(token)
        print(f"{token} = {cnt}")
        add(f"UI JS required marker {token}", cnt == 1, f"count={cnt} expected=1")

    required_present = [
        "hydrateDetectionControlsFromSettings",
        "scheduleDetectionUiHydrate",
        "persistActiveDetectionControlsToPreset",
        "schedulePersistActiveDetectionControlsToPreset",
        "operatorDetectionLimit: before.operatorDetectionLimit",
        "operatorDetectEvery: before.operatorDetectEvery",
        "operatorDetectionAreaMode: before.operatorDetectionAreaMode",
        "max_detections",
        "detect_every_n_frames",
        "detection_mode",
    ]

    for token in required_present:
        cnt = web.count(token)
        print(f"{token} = {cnt}")
        add(f"UI JS token present {token}", cnt >= 1, f"count={cnt}")

    fallback_ok = (
        "objectPresetsCustom: (before.objectPresetsCustom && Object.keys(before.objectPresetsCustom).length" in web
        or "Object.assign({}, OBJECT_PRESETS" in web
        or "settings.objectPresetsCustom = custom" in web
    )
    add(
        "UI saveSettings does not persist empty objectPresetsCustom",
        fallback_ok,
        "fallback/merge guard marker present" if fallback_ok else "missing objectPresetsCustom fallback/merge guard"
    )

    active_preset_write_ok = (
        "objectPresetsCustom" in web
        and "activeObjectPreset" in web
        and "persistActiveDetectionControlsToPreset" in web
        and "max_detections" in web
        and "detect_every_n_frames" in web
        and "detection_mode" in web
    )
    add(
        "UI detection controls persist into active object preset",
        active_preset_write_ok,
        "active preset persistence markers present" if active_preset_write_ok else "missing active preset persistence markers"
    )

    hydrate_export_ok = (
        "window.ptzCleanLayer" in web
        and "hydrateDetectionControlsFromSettings" in web
        and "rebuildPanels" in web
    )
    add(
        "UI exposes hydrate/rebuild helpers",
        hydrate_export_ok,
        "window.ptzCleanLayer hydrate/rebuild markers present" if hydrate_export_ok else "missing hydrate/rebuild export markers"
    )
'''

if FUNC_NAME not in s:
    insert_markers = [
        '\ndef audit_settings',
        '\ndef audit_layer2',
        '\ndef audit_detector',
        '\ndef audit_layer3',
    ]
    pos = -1
    marker_used = None
    for marker in insert_markers:
        pos = s.find(marker)
        if pos >= 0:
            marker_used = marker.strip()
            break
    if pos < 0:
        raise SystemExit('Could not find insertion point before settings/detector audit function')
    s = s[:pos] + new_func + s[pos:]
    print('Inserted function:', FUNC_NAME, 'before', marker_used)
else:
    print('Function already exists:', FUNC_NAME)

# Insert function call before settings audit. This is intentionally conservative and idempotent.
call = '    audit_ui_detection_controls_contract()\n'
if 'audit_ui_detection_controls_contract()' not in s.replace(new_func, ''):
    call_markers = [
        '    audit_settings()',
        '    audit_layer2()',
        '    audit_detector()',
        '    audit_layer3()',
    ]
    pos = -1
    call_marker_used = None
    for marker in call_markers:
        pos = s.find(marker)
        if pos >= 0:
            call_marker_used = marker.strip()
            break
    if pos < 0:
        raise SystemExit('Could not find main call insertion point before settings/detector audit call')
    s = s[:pos] + call + s[pos:]
    print('Inserted call before:', call_marker_used)
else:
    print('Call already exists:', FUNC_NAME)

AUDIT.write_text(s, encoding='utf-8')
print('OK patched ptz_contract_audit.py UI detection JS contract v1')
print('Backup:', backup)