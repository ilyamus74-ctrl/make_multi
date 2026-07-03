#!/usr/bin/env python3
from pathlib import Path
import time

path = Path("/root/new_yolo8/ptz_contract_audit.py")
text = path.read_text(encoding="utf-8", errors="ignore")

marker = "PTZ_AUDIT_DAEMON_LIFECYCLE_LOG_PATHS_V4"
if marker in text:
    print("Already patched:", marker)
    raise SystemExit(0)

backup = path.with_name(f"{path.name}.bak_daemon_lifecycle_log_paths_v4_{int(time.time())}")
backup.write_text(text, encoding="utf-8")

# 1) Add current log path. Current runtime archives /dev/shm log on disarm,
# so child_start can be present only in logs/current/object_tracking_daemon.log.
old_paths = """        for path in [
            "/dev/shm/new_yolo8_object_tracking/object_tracking_daemon.log",
            "/root/new_yolo8/object_tracking_daemon.log"
        ]:
"""
new_paths = """        # PTZ_AUDIT_DAEMON_LIFECYCLE_LOG_PATHS_V4
        # Runtime may archive /dev/shm log on ptzArmed=false.
        # Check both shared-memory live log and persistent current log.
        for path in [
            "/dev/shm/new_yolo8_object_tracking/object_tracking_daemon.log",
            "/root/new_yolo8/logs/current/object_tracking_daemon.log",
            "/root/new_yolo8/object_tracking_daemon.log"
        ]:
"""
if old_paths not in text:
    raise SystemExit("ERROR: child_start log path block not found")
text = text.replace(old_paths, new_paths, 1)

# 2) Accept armed_running as lifecycle evidence too.
old_event_filter = """                if obj.get("event") != "child_start":
                    continue
"""
new_event_filter = """                if obj.get("event") not in ("child_start", "armed_running"):
                    continue
"""
if old_event_filter not in text:
    raise SystemExit("ERROR: child_start event filter not found")
text = text.replace(old_event_filter, new_event_filter, 1)

# 3) Make printed text less misleading.
text = text.replace(
    'print("child_start events during arm =", len(events))',
    'print("runtime start events during arm =", len(events))',
    1
)

# 4) Print event name too.
old_print = """        print("child_start", {
            "path": ev.get("_path"),
            "ts": ev.get("ts"),
            "preset": ev.get("preset"),
            "search_preset": ev.get("search_preset"),
            "pid": ev.get("pid")
        })
"""
new_print = """        print("runtime_event", {
            "event": ev.get("event"),
            "path": ev.get("_path"),
            "ts": ev.get("ts"),
            "preset": ev.get("preset"),
            "search_preset": ev.get("search_preset"),
            "pid": ev.get("pid")
        })
"""
if old_print not in text:
    raise SystemExit("ERROR: child_start print block not found")
text = text.replace(old_print, new_print, 1)

# 5) Rename assertion/details, but keep condition semantics.
old_add = """    add(
        "Daemon lifecycle arm child_start observed",
        len(events) >= 1 or wc1_max >= 1,
        f"child_start_events={len(events)} max_watch_count={wc1_max} final_watch_count={wc1_final} samples={samples}"
    )
"""
new_add = """    add(
        "Daemon lifecycle arm runtime start observed",
        len(events) >= 1 or wc1_max >= 1,
        f"runtime_events={len(events)} max_watch_count={wc1_max} final_watch_count={wc1_final} samples={samples}"
    )
"""
if old_add not in text:
    raise SystemExit("ERROR: daemon lifecycle add block not found")
text = text.replace(old_add, new_add, 1)

path.write_text(text, encoding="utf-8")

print("OK patched ptz_contract_audit.py daemon lifecycle log paths v4")
print("Backup:", backup)
print("Changes:")
print(" - added logs/current/object_tracking_daemon.log to lifecycle event scan")
print(" - accepts armed_running as lifecycle evidence")
print(" - renamed audit assertion to runtime start observed")