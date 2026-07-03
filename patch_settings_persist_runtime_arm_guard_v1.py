#!/usr/bin/env python3
from pathlib import Path
import time

path = Path("/root/new_yolo8/settings_persist_daemon.py")
text = path.read_text(encoding="utf-8", errors="ignore")

if "PTZ_SETTINGS_PERSIST_RUNTIME_ARM_GUARD_START" in text:
    print("Already patched: PTZ_SETTINGS_PERSIST_RUNTIME_ARM_GUARD_START")
    raise SystemExit(0)

needle = '''    runtime["config_version"] = max(16, int(runtime.get("config_version") or 16))
    runtime["activeObjectPreset"] = runtime.get("activeObjectPreset") or file_data.get("activeObjectPreset") or "person_single"
    runtime["activeSearchPreset"] = runtime.get("activeSearchPreset") or file_data.get("activeSearchPreset") or "lost_step_wait"

    # В runtime не вмешиваемся, но в файл auto-arm не сохраняем.
    file_out = dict(runtime)
'''

insert = '''    runtime["config_version"] = max(16, int(runtime.get("config_version") or 16))
    runtime["activeObjectPreset"] = runtime.get("activeObjectPreset") or file_data.get("activeObjectPreset") or "person_single"
    runtime["activeSearchPreset"] = runtime.get("activeSearchPreset") or file_data.get("activeSearchPreset") or "lost_step_wait"

    # PTZ_SETTINGS_PERSIST_RUNTIME_ARM_GUARD_START
    # Runtime arm state is transient. ui_settings.json is a persistent safe-state file.
    # Do not write ui_settings.json while PTZ is armed/ptz mode, because the backend may
    # reload the file and collapse runtime ptzArmed=true back to ptzArmed=false.
    runtime_armed = bool(runtime.get("ptzArmed")) or str(runtime.get("controlMode") or "").lower() == "ptz"
    if runtime_armed:
        if api_needs_repair:
            post_json(f"{MJPEG_BASE}/api/settings", runtime, timeout=3)
            log("repaired API objectPresetsCustom from file while runtime armed")
        log(
            "skip persist: runtime armed "
            + f"ptzArmed={runtime.get('ptzArmed')} "
            + f"controlMode={runtime.get('controlMode')}"
        )
        return
    # PTZ_SETTINGS_PERSIST_RUNTIME_ARM_GUARD_END

    # В runtime не вмешиваемся, но в файл auto-arm не сохраняем.
    file_out = dict(runtime)
'''

if needle not in text:
    print("ERROR: expected insertion point not found")
    print("Needle:")
    print(needle)
    raise SystemExit(1)

backup = path.with_name(path.name + f".bak_runtime_arm_guard_v1_{int(time.time())}")
backup.write_text(text, encoding="utf-8")

text = text.replace(needle, insert, 1)
path.write_text(text, encoding="utf-8")

print("OK patched settings_persist_daemon.py runtime arm guard v1")
print(f"Backup: {backup}")
print("Changes:")
print(" - skip ui_settings.json write while runtime ptzArmed=true/controlMode=ptz")
print(" - preserve optional objectPresetsCustom API repair without collapsing arm state")