#!/usr/bin/env python3
# patch_ptz_contract_seed_v1.py
#
# Purpose:
#   Fix the remaining ptz_contract_audit.py failures where runtime is OK,
#   but ui_settings.json has no durable preset seed:
#     - custom presets non-empty
#     - ptzArmed is false
#     - activeObjectPreset points to an existing preset
#     - activeSearchPreset is initialized
#
# Usage:
#   cd /root/new_yolo8
#   python3 patch_ptz_contract_seed_v1.py --settings ./ui_settings.json
#
# Optional:
#   python3 patch_ptz_contract_seed_v1.py --settings ./ui_settings.json --no-backup

from __future__ import annotations

import argparse
import copy
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict


DEFAULT_PRESET_KEY = "contract_default_object"


DEFAULT_OBJECT_PRESET: Dict[str, Any] = {
    "name": "Contract Default Object",
    "description": "Durable seed preset required by PTZ_MASTER_CONTRACT.md",
    "version": 1,

    # Detector/runtime defaults matching the current launcher contract from audit output.
    "max_detections": 5,
    "max_raw_candidates": 10,
    "detect_every_n_frames": 5,

    # Detection area defaults.
    "detection_mode": "full_frame",
    "rois": [],

    # PTZ/autopilot safe defaults: do not arm automatically.
    "ptzArmed": False,
    "controlMode": "manual",
    "autopilot_enabled": False,

    # Common PTZ tuning defaults seen in the current runtime.
    "kp": 18,
    "ki": 0,
    "kd": 1.5,
    "deadzone": 0.05,
    "max_pan": 20,
    "max_tilt": 20,
    "max_accel": 4,
    "target_x": 0.5,
    "target_y": 0.5,
    "min_pan": 4,
    "min_tilt": 3,

    # Auto-zoom defaults from runtime, kept non-invasive.
    "auto_zoom_enable": True,
    "auto_zoom_target_h": 0.68,
    "auto_zoom_deadzone": 0.08,
}


def load_json(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"settings file does not exist: {path}")
    raw = path.read_text(encoding="utf-8").strip()
    if not raw:
        return {}
    data = json.loads(raw)
    if not isinstance(data, dict):
        raise TypeError(f"{path} must contain a JSON object, got {type(data).__name__}")
    return data


def dump_json(path: Path, data: Dict[str, Any]) -> None:
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2, sort_keys=False) + "\n",
        encoding="utf-8",
    )


def ensure_dict(root: Dict[str, Any], key: str) -> Dict[str, Any]:
    val = root.get(key)
    if not isinstance(val, dict):
        root[key] = {}
    return root[key]


def deep_merge_missing(dst: Dict[str, Any], src: Dict[str, Any]) -> bool:
    changed = False
    for k, v in src.items():
        if k not in dst:
            dst[k] = copy.deepcopy(v)
            changed = True
        elif isinstance(dst.get(k), dict) and isinstance(v, dict):
            if deep_merge_missing(dst[k], v):
                changed = True
    return changed


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--settings", default="./ui_settings.json", help="Path to ui_settings.json")
    ap.add_argument("--preset-key", default=DEFAULT_PRESET_KEY, help="Preset key to seed/use")
    ap.add_argument("--no-backup", action="store_true", help="Do not create timestamped backup")
    args = ap.parse_args()

    settings_path = Path(args.settings).expanduser().resolve()
    preset_key = args.preset_key

    data = load_json(settings_path)
    before = copy.deepcopy(data)
    changed = False

    # Required top-level durable state.
    if data.get("ptzArmed") is not False:
        data["ptzArmed"] = False
        changed = True

    if data.get("controlMode") != "manual":
        data["controlMode"] = "manual"
        changed = True

    # Canonical custom preset container used by the contract/audit.
    custom = ensure_dict(data, "customPresets")
    if "customPresets" not in before or not isinstance(before.get("customPresets"), dict):
        changed = True

    preset = custom.get(preset_key)
    if not isinstance(preset, dict):
        custom[preset_key] = copy.deepcopy(DEFAULT_OBJECT_PRESET)
        changed = True
    else:
        if deep_merge_missing(preset, DEFAULT_OBJECT_PRESET):
            changed = True

    # Make active preset durable and resolvable.
    if data.get("activeObjectPreset") != preset_key:
        data["activeObjectPreset"] = preset_key
        changed = True

    # Search preset is kept initialized. Many existing UIs use the same custom preset map.
    if data.get("activeSearchPreset") in (None, "", "null"):
        data["activeSearchPreset"] = preset_key
        changed = True

    # Compatibility aliases for older UI/API code. These are harmless if ignored.
    if "customObjectPresets" in data and isinstance(data["customObjectPresets"], dict):
        if preset_key not in data["customObjectPresets"]:
            data["customObjectPresets"][preset_key] = copy.deepcopy(custom[preset_key])
            changed = True

    if not changed:
        print(f"OK: no changes needed: {settings_path}")
        return 0

    if not args.no_backup:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = settings_path.with_name(settings_path.name + f".before_contract_seed.{ts}")
        backup.write_text(json.dumps(before, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"Backup: {backup}")

    dump_json(settings_path, data)
    print(f"Updated: {settings_path}")
    print(f"activeObjectPreset={data.get('activeObjectPreset')}")
    print(f"activeSearchPreset={data.get('activeSearchPreset')}")
    print(f"ptzArmed={data.get('ptzArmed')}")
    print(f"customPresets keys={list(data.get('customPresets', {}).keys())}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)