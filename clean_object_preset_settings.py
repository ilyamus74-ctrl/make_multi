from pathlib import Path
import json
import time

p = Path("ui_settings.json")

if not p.exists():
    raise SystemExit("ui_settings.json not found")

s = json.loads(p.read_text(encoding="utf-8", errors="replace"))

bak = p.with_suffix(p.suffix + f".bak_clean_object_presets_{int(time.time())}")
bak.write_text(json.dumps(s, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

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

custom = s.get("objectPresetsCustom")

if not isinstance(custom, dict):
    custom = {}

for name, preset in list(custom.items()):
    if not isinstance(preset, dict):
        continue

    ptz = preset.get("ptz")

    if isinstance(ptz, dict):
        preset["ptz"] = {
            k: v
            for k, v in ptz.items()
            if k in allowed_ptz
        }

    if "tracking_mode" not in preset:
        if name in ["people", "cars", "airplanes", "birds"]:
            preset["tracking_mode"] = "multi_operator"
        else:
            preset["tracking_mode"] = "single_auto"

    if "loss_behavior" not in preset:
        if preset.get("tracking_mode") == "multi_operator":
            preset["loss_behavior"] = "operator_select"
        else:
            preset["loss_behavior"] = "continuous_wide_scan_x"

s["objectPresetsCustom"] = custom
s["ptzArmed"] = False
s["controlMode"] = "manual"

if not s.get("activeObjectPreset"):
    s["activeObjectPreset"] = "person_single"

if not s.get("activeSearchPreset"):
    s["activeSearchPreset"] = "lost_step_wait"

p.write_text(json.dumps(s, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

print("OK cleaned ui_settings.json")
print("Backup:", bak)
print("activeObjectPreset =", s.get("activeObjectPreset"))
print("activeSearchPreset =", s.get("activeSearchPreset"))
print("ptzArmed =", s.get("ptzArmed"))
print("custom keys =", sorted(custom.keys()))
