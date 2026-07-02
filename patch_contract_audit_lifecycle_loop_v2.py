from pathlib import Path
import time

p = Path("ptz_contract_audit.py")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_lifecycle_loop_v2_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

s = s.replace(
    'test_settings["activeSearchPreset"] = test_settings.get("activeSearchPreset") or "lost_step_wait"',
    'test_settings["activeSearchPreset"] = "lost_wide_cycle"'
)

s = s.replace(
    's["ptzArmed"] = bool(value)\n        s["controlMode"] = "ptz" if value else "manual"',
    's["activeObjectPreset"] = "car_single"\n        s["activeSearchPreset"] = "lost_wide_cycle"\n        s["objectPresetTrackingMode"] = "single_auto"\n        s["objectPresetLossBehavior"] = "continuous_wide_scan_x"\n        s["ptzArmed"] = bool(value)\n        s["controlMode"] = "ptz" if value else "manual"'
)

p.write_text(s, encoding="utf-8")

print("OK patched lifecycle audit to use lost_wide_cycle")
print("Backup:", bak)
