from pathlib import Path
import json
import time

p = Path("ui_settings.json")

if not p.exists():
    raise SystemExit("ui_settings.json not found")

data = json.loads(p.read_text(encoding="utf-8", errors="replace"))

bak = p.with_suffix(p.suffix + f".bak_clean_object_presets_v2_{int(time.time())}")
bak.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

allowed_ptz = {
    "target_x",
    "target_y",
    "auto_zoom_enable",
    "auto_zoom_target_h",
    "auto_zoom_deadzone",
    "auto_zoom_cmd",
    "auto_zoom_sign",
    "auto_zoom_period_ms",
    "zoom_scale_enable",
    "zoom_scale_min",
    "zoom_scale_max",
    "zoom_scale_smoothing"
}

custom = data.get("objectPresetsCustom")

if not isinstance(custom, dict):
    custom = {}

for name, preset in custom.items():
    if not isinstance(preset, dict):
        continue

    ptz = preset.get("ptz")

    if isinstance(ptz, dict):
        preset["ptz"] = {
            k: v
            for k, v in ptz.items()
            if k in allowed_ptz
        }

    if not preset.get("tracking_mode"):
        preset["tracking_mode"] = "multi_operator" if name in ["people", "cars", "airplanes", "birds"] else "single_auto"

    if not preset.get("loss_behavior"):
        preset["loss_behavior"] = "operator_select" if preset.get("tracking_mode") == "multi_operator" else "continuous_wide_scan_x"

data["objectPresetsCustom"] = custom
data["ptzArmed"] = False
data["controlMode"] = "manual"

if not data.get("activeObjectPreset"):
    data["activeObjectPreset"] = "person_single"

if not data.get("activeSearchPreset"):
    data["activeSearchPreset"] = "lost_step_wait"

p.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

print("OK cleaned ui_settings.json")
print("Backup:", bak)
print("activeObjectPreset =", data.get("activeObjectPreset"))
print("activeSearchPreset =", data.get("activeSearchPreset"))
print("ptzArmed =", data.get("ptzArmed"))
print("controlMode =", data.get("controlMode"))
print("custom keys =", sorted(custom.keys()))
