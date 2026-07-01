from pathlib import Path
import time

p = Path("apply_ptz_object_preset.py")
s = p.read_text(encoding="utf-8")

bak = p.with_suffix(p.suffix + f".bak_do_not_override_speed_profile_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

# Add helper before apply_preset_config.
anchor = "\ndef apply_preset_config("
helper = r'''
def filter_ptz_framing_config(ptz):
    """
    Object presets are NOT speed profiles.

    Keep only framing and auto-zoom fields here.
    PTZ speed/dynamics must come from PTZ SPEED TUNE zoom samples:
    kp/kd/deadzone/max_pan/max_tilt/max_accel/min_pan/min_tilt/etc.
    """
    allowed = {
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

    return {
        k: v
        for k, v in dict(ptz or {}).items()
        if k in allowed
    }


'''
if "def filter_ptz_framing_config(" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: apply_preset_config anchor not found")
    s = s.replace(anchor, "\n" + helper + anchor, 1)

# Replace ptz post in apply_preset_config.
old = '''    ptz_res = {}
    if ptz:
        ptz_res = post_json(f"{AUTOPILOT_BASE}/api/autopilot/config", ptz)
'''

new = '''    ptz_res = {}
    ptz_framing = filter_ptz_framing_config(ptz)

    if ptz_framing:
        ptz_res = post_json(f"{AUTOPILOT_BASE}/api/autopilot/config", ptz_framing)

    # Re-apply speed profile nearest/current after framing update.
    # This keeps object preset from overriding PTZ SPEED TUNE zoom-sample dynamics.
    try:
        post_json(f"{AUTOPILOT_BASE}/api/autopilot/speed_profile/apply_nearest", {}, timeout=2)
    except Exception as e:
        event_log("speed_profile_apply_nearest_failed", error=str(e))
'''

if old not in s:
    raise SystemExit("ERROR: ptz post block not found")

s = s.replace(old, new, 1)

# In report, keep original ptz visible, but add actual sent config.
old = '''        "ptz": ptz,
        "detector": det_res,
'''

new = '''        "ptz": ptz,
        "ptz_framing_sent": ptz_framing,
        "detector": det_res,
'''

if old in s:
    s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")

print("OK patched apply_ptz_object_preset.py")
print("Backup:", bak)
