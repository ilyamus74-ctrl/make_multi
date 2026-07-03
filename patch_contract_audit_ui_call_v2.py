#!/usr/bin/env python3
from pathlib import Path
import time
import re

ROOT = Path("/root/new_yolo8")
path = ROOT / "ptz_contract_audit.py"

s = path.read_text(encoding="utf-8", errors="ignore")
backup = path.with_suffix(path.suffix + f".bak_ui_js_call_v2_{int(time.time())}")
backup.write_text(s, encoding="utf-8")

changed = False

func = """
def audit_ui_detection_controls_contract():
    section("LAYER 1B — UI DETECTION CONTROLS JS CONTRACT")

    web_path = ROOT / "web/index.html"
    try:
        js = web_path.read_text(encoding="utf-8", errors="ignore")
    except Exception as e:
        add("UI detection JS readable", False, str(e))
        return

    required = [
        "PTZ_CLEAN_DETECTION_UI_HYDRATE_F5_START",
        "PTZ_CLEAN_DETECTION_ACTIVE_PRESET_PERSIST_START",
        "hydrateDetectionControlsFromSettings",
        "scheduleDetectionUiHydrate",
        "persistActiveDetectionControlsToPreset",
        "schedulePersistActiveDetectionControlsToPreset",
        "PTZ_SELECT_DYNAMIC_OPTION_FIX_START",
    ]

    for token in required:
        add(f"UI detection JS marker {token}", token in js, f"count={js.count(token)}")

    persist_fields = [
        "max_detections",
        "detect_every_n_frames",
        "detection_mode",
        "operatorDetectionLimit",
        "operatorDetectEvery",
        "operatorDetectionAreaMode",
        "objectPresetsCustom",
        "activeObjectPreset",
    ]

    for token in persist_fields:
        add(f"UI detection persistence token {token}", token in js, f"count={js.count(token)}")

    bad_empty_posts = [
        "objectPresetsCustom: {}",
        "objectPresetsCustom={}",
        "objectPresetsCustom = {}",
    ]

    for token in bad_empty_posts:
        add(f"UI must not hard-code empty {token}", token not in js, f"count={js.count(token)}")

"""

if "def audit_ui_detection_controls_contract():" not in s:
    marker = "\ndef audit_settings("
    if marker not in s:
        raise SystemExit("Could not find def audit_settings insertion point")
    s = s.replace(marker, func + marker, 1)
    changed = True
    print("Inserted function: audit_ui_detection_controls_contract")
else:
    print("Function already exists: audit_ui_detection_controls_contract")

body_without_def_name = s.replace("def audit_ui_detection_controls_contract():", "")
if "audit_ui_detection_controls_contract()" not in body_without_def_name:
    lines = s.splitlines()
    out = []
    inserted = False

    for line in lines:
        if not inserted and re.match(r"^(\s*)audit_settings\(\)\s*$", line):
            indent = re.match(r"^(\s*)", line).group(1)
            out.append(indent + "audit_ui_detection_controls_contract()")
            inserted = True
            changed = True
            print("Inserted call before audit_settings()")
        out.append(line)

    if not inserted:
        raise SystemExit("Could not find audit_settings() call insertion point")

    s = "\n".join(out) + "\n"
else:
    print("Call already exists: audit_ui_detection_controls_contract()")

path.write_text(s, encoding="utf-8")

print("Backup:", backup)
print("Changed:", changed)
print("OK")