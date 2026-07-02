from pathlib import Path
import time

p = Path("ptz_contract_audit.py")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_lifecycle_childstart_log_v4_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

start = s.find("def audit_daemon_lifecycle():")
end = s.find("\ndef audit_launcher():", start)

if start < 0 or end < 0:
    raise SystemExit("function boundaries not found")

new_func = r'''def audit_daemon_lifecycle():
    section("LAYER 5/6 — DAEMON AND RUNTIME LIFECYCLE")

    settings = get_json(f"{MJPEG_BASE}/api/settings")

    if "__error__" in settings:
        add("Lifecycle skipped settings reachable", False, settings["__error__"])
        return

    original = dict(settings)

    def watch_count():
        return len([
            x
            for x in find_processes("apply_ptz_object_preset.py")
            if "--watch" in x[1]
        ])

    def child_start_events_since(ts0):
        events = []
        paths = [
            ROOT / "object_tracking_daemon.log",
            Path("/dev/shm/new_yolo8_object_tracking/object_tracking_daemon.log")
        ]

        for path in paths:
            if not path.exists():
                continue

            try:
                lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
            except Exception:
                continue

            for line in lines:
                line = line.strip()

                if not line:
                    continue

                try:
                    ev = json.loads(line)
                except Exception:
                    continue

                if ev.get("event") != "child_start":
                    continue

                try:
                    ev_ts = float(ev.get("ts") or 0)
                except Exception:
                    ev_ts = 0

                if ev_ts + 0.001 < ts0:
                    continue

                events.append({
                    "path": str(path),
                    "ts": ev_ts,
                    "preset": ev.get("preset"),
                    "search_preset": ev.get("search_preset"),
                    "pid": ev.get("pid")
                })

        # Logs are duplicated in current setup, so dedupe by timestamp/pid/preset.
        dedup = []
        seen = set()

        for ev in sorted(events, key=lambda x: (x.get("ts") or 0, str(x.get("path")))):
            key = (
                round(float(ev.get("ts") or 0), 3),
                ev.get("pid"),
                ev.get("preset"),
                ev.get("search_preset")
            )

            if key in seen:
                continue

            seen.add(key)
            dedup.append(ev)

        return dedup

    def set_armed(value):
        s = get_json(f"{MJPEG_BASE}/api/settings")
        custom = s.get("objectPresetsCustom") or {}
        car = custom.get("car_single") or {}

        s["activeObjectPreset"] = "car_single"
        s["activeSearchPreset"] = s.get("activeSearchPreset") or "lost_step_wait"
        s["objectPresetTrackingMode"] = "single_auto"
        s["objectPresetLossBehavior"] = "continuous_wide_scan_x"
        s["lastAppliedObjectPreset"] = {
            "name": "car_single",
            "label": car.get("label") or "МАШИНА",
            "tracking_mode": "single_auto",
            "loss_behavior": "continuous_wide_scan_x",
            "ts": int(time.time())
        }
        s["ptzArmed"] = bool(value)
        s["controlMode"] = "ptz" if value else "manual"
        post_json(f"{MJPEG_BASE}/api/settings", s)

    print("Step A: disarm")
    set_armed(False)
    post_json(f"{PTZ_BASE}/api/autopilot/stop", {})
    time.sleep(3)
    wc0 = watch_count()
    print("watch_count after disarm =", wc0)

    print("Step B: arm")
    t_arm = time.time()
    set_armed(True)

    samples = []

    for i in range(12):
        time.sleep(0.5)
        wc = watch_count()
        samples.append(wc)
        print("watch_count arm sample", i + 1, "=", wc)

    child_events = child_start_events_since(t_arm - 0.25)
    wc1_max = max(samples) if samples else 0
    wc1_final = samples[-1] if samples else 0

    print("watch_count max during arm =", wc1_max)
    print("watch_count final during arm =", wc1_final)
    print("child_start events during arm =", len(child_events))

    for ev in child_events[-5:]:
        print("child_start", ev)

    print("Step C: final disarm")
    set_armed(False)
    post_json(f"{PTZ_BASE}/api/autopilot/stop", {})
    time.sleep(4)
    wc2 = watch_count()
    print("watch_count after final disarm =", wc2)

    restored = dict(original)
    restored["ptzArmed"] = False
    restored["controlMode"] = "manual"
    post_json(f"{MJPEG_BASE}/api/settings", restored)
    post_json(f"{PTZ_BASE}/api/autopilot/stop", {})

    add("Daemon lifecycle disarm watch_count=0", wc0 == 0, f"watch_count={wc0}")
    add(
        "Daemon lifecycle arm child_start observed",
        len(child_events) >= 1,
        f"child_start_events={len(child_events)} max_watch_count={wc1_max} final_watch_count={wc1_final} samples={samples}"
    )
    add("Daemon lifecycle final disarm watch_count=0", wc2 == 0, f"watch_count={wc2}")

'''

s = s[:start] + new_func + s[end:]

p.write_text(s, encoding="utf-8")

print("OK patched ptz_contract_audit.py lifecycle child_start log rule")
print("Backup:", bak)