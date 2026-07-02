from pathlib import Path
import time

p = Path("apply_ptz_object_preset.py")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_model_in_object_preset_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

old = '''    out["operatorDetectionAreaMode"] = str(preset.get("detection_mode") or "full_frame")
    out["objectPresetTrackingMode"] = str(preset.get("tracking_mode") or "manual_select")
'''

new = '''    out["operatorDetectionAreaMode"] = str(preset.get("detection_mode") or "full_frame")

    model = (
        preset.get("model")
        or preset.get("operatorModel")
        or preset.get("current_model")
        or preset.get("model_path")
        or ""
    )

    if model:
        out["operatorModel"] = str(model)

    out["objectPresetTrackingMode"] = str(preset.get("tracking_mode") or "manual_select")
'''

if old in s:
    s = s.replace(old, new, 1)
else:
    print("WARN: merge_settings model anchor not found")

old = '''    ptz = dict(preset.get("ptz") or {})

    wait_api(f"{MJPEG_BASE}/api/ping", attempts=30, delay=0.25)
'''

new = '''    ptz = dict(preset.get("ptz") or {})
    model = (
        preset.get("model")
        or preset.get("operatorModel")
        or preset.get("current_model")
        or preset.get("model_path")
        or ""
    )
    model = str(model or "").strip()

    wait_api(f"{MJPEG_BASE}/api/ping", attempts=30, delay=0.25)
'''

if old in s:
    s = s.replace(old, new, 1)
else:
    print("WARN: apply_preset_config model anchor not found")

old = '''    det_res = post_json(f"{MJPEG_BASE}/api/detector/config", {
        "detect_enabled": True,
        "selected_classes": classes
    })
'''

new = '''    det_payload = {
        "detect_enabled": True,
        "selected_classes": classes
    }

    if model:
        det_payload["current_model"] = model

    det_res = post_json(f"{MJPEG_BASE}/api/detector/config", det_payload)
'''

if old in s:
    s = s.replace(old, new, 1)
else:
    print("WARN: detector config payload anchor not found")

old = '''        "classes": classes,
        "detection_mode": mode,
'''

new = '''        "classes": classes,
        "model": model,
        "detection_mode": mode,
'''

if old in s:
    s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")

print("OK patched apply_ptz_object_preset.py")
print("Backup:", bak)
