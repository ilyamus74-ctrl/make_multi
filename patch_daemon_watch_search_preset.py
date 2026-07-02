from pathlib import Path
import time

p = Path("object_tracking_daemon.py")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_watch_search_preset_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

helper = r'''
def active_search_preset_name(settings):
    return str(settings.get("activeSearchPreset") or "lost_step_wait")


'''
if "def active_search_preset_name(" not in s:
    anchor = "\ndef is_single_auto("
    if anchor not in s:
        raise SystemExit("ERROR: is_single_auto anchor not found")
    s = s.replace(anchor, "\n" + helper + anchor, 1)

s = s.replace(
    "def start_child(preset_name):",
    "def start_child(preset_name, search_preset='lost_step_wait'):"
)

s = s.replace(
    "    child_preset = preset_name\n\n    log(\"child_start\", preset=preset_name, pid=child.pid, cmd=cmd)\n",
    "    child_preset = preset_name + '::' + str(search_preset)\n\n    log(\"child_start\", preset=preset_name, search_preset=search_preset, pid=child.pid, cmd=cmd)\n"
)

s = s.replace(
    "        preset_name = active_preset_name(settings, presets_root)\n        preset = preset_by_name(preset_name, settings, presets_root)\n",
    "        preset_name = active_preset_name(settings, presets_root)\n        search_preset = active_search_preset_name(settings)\n        preset = preset_by_name(preset_name, settings, presets_root)\n"
)

s = s.replace(
    "            if child_preset != preset_name:\n                stop_child(\"preset_changed\")\n",
    "            expected_child_key = preset_name + '::' + str(search_preset)\n            if child_preset != expected_child_key:\n                stop_child(\"preset_or_search_preset_changed\")\n"
)

s = s.replace(
    "                log(\"armed_running\", preset=preset_name, pid=child.pid)\n",
    "                log(\"armed_running\", preset=preset_name, search_preset=search_preset, pid=child.pid)\n"
)

s = s.replace(
    "        start_child(preset_name)\n",
    "        start_child(preset_name, search_preset)\n"
)

p.write_text(s, encoding="utf-8")

print("OK patched object_tracking_daemon.py")
print("Backup:", bak)
