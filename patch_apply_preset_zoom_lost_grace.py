from pathlib import Path
import time

p = Path("apply_ptz_object_preset.py")
s = p.read_text(encoding="utf-8")

bak = p.with_suffix(p.suffix + f".bak_zoom_lost_grace_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

start = s.find("def zoom_wide_pulse(")
if start < 0:
    raise SystemExit("ERROR: zoom_wide_pulse not found")

end = s.find("\ndef manual_search_pulse(", start)
if end < 0:
    raise SystemExit("ERROR: manual_search_pulse marker not found")

new_zoom_func = r'''def zoom_state():
    try:
        return get_json(f"{MJPEG_BASE}/api/zoom/state", timeout=2)
    except Exception as e:
        return {"ok": False, "error": str(e)}


def zoom_is_already_wide(skip_ratio=0.06):
    st = zoom_state()

    ratio = None
    for key in ["zoom_ratio", "ratio", "state", "zoomState"]:
        if key in st:
            try:
                ratio = float(st.get(key))
                break
            except Exception:
                pass

    sample = None
    for key in ["active_zoom_sample_idx", "sample", "sample_idx", "step"]:
        if key in st:
            try:
                sample = int(st.get(key))
                break
            except Exception:
                pass

    if ratio is not None and ratio <= float(skip_ratio):
        return True, st

    if sample is not None and sample <= 0:
        return True, st

    return False, st


def zoom_wide_pulse(reason="search", hold_ms=220, cooldown_sec=8.0, skip_ratio=0.06):
    now = time.time()
    last_ts = getattr(zoom_wide_pulse, "_last_ts", 0.0)

    if now - last_ts < float(cooldown_sec):
        event_log(
            "zoom_wide_skip_cooldown",
            reason=reason,
            cooldown_sec=cooldown_sec,
            age_sec=round(now - last_ts, 3)
        )
        return {
            "ok": True,
            "skipped": True,
            "reason": "cooldown",
            "age_sec": round(now - last_ts, 3)
        }

    already_wide, state = zoom_is_already_wide(skip_ratio=skip_ratio)

    if already_wide:
        zoom_wide_pulse._last_ts = now
        event_log(
            "zoom_wide_skip_already_wide",
            reason=reason,
            skip_ratio=skip_ratio,
            state=state
        )
        return {
            "ok": True,
            "skipped": True,
            "reason": "already_wide",
            "state": state
        }

    try:
        settings = {}
        try:
            settings = get_json(f"{MJPEG_BASE}/api/zoom_calibration/settings", timeout=2)
        except Exception:
            settings = {}

        cmd_abs = int(settings.get("cmd_abs") or 12)
        wide_sign = int(settings.get("wide_cmd_sign") or -1)
        cmd = wide_sign * max(1, min(34, cmd_abs))
        hold_ms = max(80, min(2000, int(hold_ms)))

        post_json(f"{MJPEG_BASE}/api/zoom/jog", {
            "action": "start",
            "cmd": cmd,
            "hold_ms": hold_ms,
            "source": f"auto_arm_{reason}"
        }, timeout=2)

        time.sleep((hold_ms + 80) / 1000.0)

        try:
            post_json(f"{MJPEG_BASE}/api/zoom/jog", {
                "action": "stop",
                "cmd": 0,
                "source": f"auto_arm_{reason}_stop"
            }, timeout=2)
        except Exception:
            pass

        zoom_wide_pulse._last_ts = now

        event_log("zoom_wide_pulse", reason=reason, ok=True, cmd=cmd, hold_ms=hold_ms)
        return {"ok": True, "cmd": cmd, "hold_ms": hold_ms}

    except Exception as e:
        event_log("zoom_wide_pulse", reason=reason, ok=False, error=str(e))
        return {"ok": False, "error": str(e)}

'''

s = s[:start] + new_zoom_func + s[end:]

start = s.find("def run_continuous_wide_scan_x(")
if start < 0:
    raise SystemExit("ERROR: run_continuous_wide_scan_x not found")

end = s.find("\ndef run_custom_search_plan(", start)
if end < 0:
    raise SystemExit("ERROR: run_custom_search_plan marker not found")

new_scan_func = r'''def run_continuous_wide_scan_x(classes, preset_name, plan, max_seconds=0):
    stop_ptz()

    lost_seen_since = None
    last_status = None
    cycle = 0

    lost_grace_sec = float(plan.get("lost_grace_sec") or 1.8)
    lost_grace_poll_sec = float(plan.get("lost_grace_poll_sec") or 0.25)

    zoom_cooldown = float(plan.get("zoom_wide_cooldown_sec") or 8.0)
    zoom_skip_ratio = float(plan.get("zoom_wide_skip_ratio") or 0.06)
    zoom_hold_ms = int(plan.get("zoom_wide_hold_ms") or 140)

    if plan.get("zoom_wide_on_start", True):
        zoom_wide_pulse(
            "scan_start",
            hold_ms=zoom_hold_ms,
            cooldown_sec=zoom_cooldown,
            skip_ratio=zoom_skip_ratio
        )

    deadline = None
    if max_seconds and float(max_seconds) > 0:
        deadline = time.time() + float(max_seconds)

    event_log("continuous_scan_start", preset=preset_name, classes=classes)

    while True:
        if deadline and time.time() >= deadline:
            stop_ptz()
            out = {
                "ok": False,
                "mode": "continuous_wide_scan_x",
                "reason": "max_seconds_reached",
                "preset": preset_name
            }
            event_log("continuous_scan_finished", **out)
            write_state(mode="stopped", preset=preset_name, result=out)
            return out

        try:
            tr = get_json(f"{MJPEG_BASE}/api/tracker/state", timeout=3)
            ap = get_json(f"{AUTOPILOT_BASE}/api/autopilot/state", timeout=3)

            tracking_ok = (
                ap.get("enabled") is True
                and tr.get("mode") == "TRACKING"
                and tr.get("selected_box_valid") is True
            )

            if tracking_ok:
                lost_seen_since = None

                status = {
                    "mode": "tracking",
                    "preset": preset_name,
                    "track_id": tr.get("selected_track_id"),
                    "ptz": ap.get("mode"),
                    "cmd_pan": ap.get("cmd_pan"),
                    "cmd_tilt": ap.get("cmd_tilt"),
                    "zoom_cmd": ap.get("auto_zoom_cmd")
                }

                if status != last_status:
                    event_log("tracking", **status)
                    write_state(**status)
                    last_status = status

                time.sleep(0.5)
                continue

            # If PTZ is active but tracker briefly lost target, do not instantly stop/reacquire.
            # Give the internal tracker/autopilot a short grace period.
            if ap.get("enabled") is True:
                now = time.time()

                if lost_seen_since is None:
                    lost_seen_since = now
                    event_log(
                        "lost_grace_start",
                        preset=preset_name,
                        tracker=tr,
                        autopilot={
                            "enabled": ap.get("enabled"),
                            "mode": ap.get("mode"),
                            "last_tracker_mode": ap.get("last_tracker_mode")
                        }
                    )

                lost_age = now - lost_seen_since

                if lost_age < lost_grace_sec:
                    write_state(
                        mode="lost_grace",
                        preset=preset_name,
                        lost_age=round(lost_age, 3),
                        track_id=tr.get("selected_track_id")
                    )
                    time.sleep(lost_grace_poll_sec)
                    continue

                event_log(
                    "lost_object",
                    preset=preset_name,
                    lost_age=round(lost_age, 3),
                    tracker=tr,
                    autopilot=ap
                )

                stop_ptz()
                lost_seen_since = None

                if plan.get("zoom_wide_on_lost", True):
                    zoom_wide_pulse(
                        "lost",
                        hold_ms=zoom_hold_ms,
                        cooldown_sec=zoom_cooldown,
                        skip_ratio=zoom_skip_ratio
                    )

            acquire = try_acquire_once(classes, tracker_wait_sec=float(plan.get("tracker_wait_sec") or 4.5))

            if acquire.get("ok"):
                write_state(mode="tracking", preset=preset_name, result=acquire)
                cycle = 0
                continue

            step = scan_step_default(plan, cycle)
            repeat = max(1, int(step.get("repeat") or 1))
            pan = float(step.get("pan") or plan.get("pan_norm") or 0.55)
            tilt = float(step.get("tilt") or plan.get("tilt_norm") or 0.0)
            pulse_ms = int(step.get("pulse_ms") or plan.get("pan_pulse_ms") or 90)

            event_log(
                "scan_step",
                preset=preset_name,
                cycle=cycle,
                step=step.get("name") or f"step_{cycle}",
                pan=pan,
                tilt=tilt,
                repeat=repeat
            )

            if plan.get("zoom_wide_every_cycles", 0):
                n = int(plan.get("zoom_wide_every_cycles") or 0)
                if n > 0 and cycle > 0 and cycle % n == 0:
                    zoom_wide_pulse(
                        "scan_cycle",
                        hold_ms=zoom_hold_ms,
                        cooldown_sec=zoom_cooldown,
                        skip_ratio=zoom_skip_ratio
                    )

            for _ in range(repeat):
                manual_search_pulse(pan=pan, tilt=tilt, pulse_ms=pulse_ms)
                acquire = try_acquire_once(classes, tracker_wait_sec=float(plan.get("tracker_wait_sec") or 4.5))

                if acquire.get("ok"):
                    write_state(mode="tracking", preset=preset_name, result=acquire)
                    break

            cycle += 1

        except KeyboardInterrupt:
            stop_ptz()
            out = {"ok": False, "mode": "continuous_wide_scan_x", "reason": "interrupted"}
            event_log("continuous_scan_interrupted", preset=preset_name)
            write_state(mode="stopped", preset=preset_name, result=out)
            return out

        except Exception as e:
            event_log("continuous_scan_exception", preset=preset_name, error=str(e))
            write_state(mode="search_error", preset=preset_name, error=str(e))
            time.sleep(0.5)

'''

s = s[:start] + new_scan_func + s[end:]

p.write_text(s, encoding="utf-8")

print("OK patched apply_ptz_object_preset.py")
print("Backup:", bak)
