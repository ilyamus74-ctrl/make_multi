from pathlib import Path
import time

p = Path("ptz_contract_audit.py")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_lifecycle_single_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

old = """    original = dict(settings)

    def watch_count():
"""

new = """    original = dict(settings)

    # Lifecycle contract is tested on single_auto preset.
    # Multi/operator presets may be armed without starting auto-watch.
    test_settings = dict(settings)
    test_settings["activeObjectPreset"] = "car_single"
    test_settings["objectPresetTrackingMode"] = "single_auto"
    test_settings["objectPresetLossBehavior"] = "continuous_wide_scan_x"
    test_settings["activeSearchPreset"] = test_settings.get("activeSearchPreset") or "lost_step_wait"
    test_settings["ptzArmed"] = False
    test_settings["controlMode"] = "manual"
    post_json(f"{MJPEG_BASE}/api/settings", test_settings)

    def watch_count():
"""

if old not in s:
    raise SystemExit("anchor not found")

s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")

print("OK patched ptz_contract_audit.py lifecycle test")
print("Backup:", bak)
