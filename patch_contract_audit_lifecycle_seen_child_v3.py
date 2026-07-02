from pathlib import Path
import time

p = Path("ptz_contract_audit.py")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_lifecycle_seen_child_v3_{int(time.time())}")
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

    def set_armed(value):
        s = get_json(f"{MJPEG_BASE}/api/settings")
        s["activeObjectPreset"] = "car_single"
        s["activeSearchPreset"] = s.get("activeSearchPreset") or "lost_step_wait"
        s["objectPresetTrackingMode"] = "single_auto"
        s["objectPresetLossBehavior"] = "continuous_wide_scan_x"
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
    set_armed(True)

    samples = []

    for i in range(8):
        time.sleep(1)
        wc = watch_count()
        samples.append(wc)
        print("watch_count arm sample", i + 1, "=", wc)

    wc1_max = max(samples) if samples else 0
    wc1_final = samples[-1] if samples else 0

    print("watch_count max during arm =", wc1_max)
    print("watch_count final during arm =", wc1_final)

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
        "Daemon lifecycle arm child observed",
        wc1_max >= 1,
        f"max_watch_count={wc1_max} final_watch_count={wc1_final} samples={samples}"
    )
    add("Daemon lifecycle final disarm watch_count=0", wc2 == 0, f"watch_count={wc2}")

'''

s = s[:start] + new_func + s[end:]

p.write_text(s, encoding="utf-8")

print("OK patched ptz_contract_audit.py lifecycle child-observed rule")
print("Backup:", bak)
