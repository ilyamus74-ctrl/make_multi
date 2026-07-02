from pathlib import Path
import json
import time

root = Path(".")
apply_path = root / "apply_ptz_object_preset.py"
preset_path = root / "ptz_search_presets.json"

# 1. Write default search presets.
search_presets = {
    "version": 1,
    "active": "lost_step_wait",
    "presets": {
        "lost_step_wait": {
            "label": "Lost: step wide, sweep, wait",
            "description": "При потере цели: WIDE -1 frame, поворот в сторону ухода, возврат, TELE +1 frame, ждём другую цель.",
            "loop": False,
            "try_acquire_after_each_step": True,
            "steps": [
                {"type": "zoom", "delta_frames": -1, "name": "wide_back_1"},
                {"type": "ptz", "direction": "last", "pan_norm": 0.55, "tilt_norm": 0.0, "mode": "hold", "hold_ms": 500, "pulse_ms": 100, "name": "pan_last_short"},
                {"type": "pause", "sec": 0.35, "name": "pause"},
                {"type": "ptz", "direction": "opposite", "pan_norm": 0.55, "tilt_norm": 0.0, "mode": "hold", "hold_ms": 500, "pulse_ms": 100, "name": "pan_back_short"},
                {"type": "zoom", "delta_frames": 1, "name": "tele_forward_1"},
                {"type": "acquire", "attempts": 12, "delay_sec": 0.25, "name": "wait_target"}
            ]
        },
        "lost_wide_cycle": {
            "label": "Lost: wide 3 frames cycle",
            "description": "При потере цели: WIDE -3 frames, длинный sweep в сторону ухода, возврат, TELE +2 frames, цикл поиска.",
            "loop": True,
            "try_acquire_after_each_step": True,
            "steps": [
                {"type": "zoom", "delta_frames": -3, "name": "wide_back_3"},
                {"type": "ptz", "direction": "last", "pan_norm": 0.65, "tilt_norm": 0.0, "mode": "hold", "hold_ms": 2000, "pulse_ms": 100, "name": "pan_last_long"},
                {"type": "pause", "sec": 1.0, "name": "pause_1s"},
                {"type": "ptz", "direction": "opposite", "pan_norm": 0.65, "tilt_norm": 0.0, "mode": "hold", "hold_ms": 2000, "pulse_ms": 100, "name": "pan_back_long"},
                {"type": "zoom", "delta_frames": 2, "name": "tele_forward_2"},
                {"type": "acquire", "attempts": 12, "delay_sec": 0.25, "name": "wait_target"}
            ]
        },
        "lost_nested_loop": {
            "label": "Lost: short + preset1 + pause",
            "description": "При потере цели: короткий шаг, затем выполняет preset1, пауза 5 сек и повтор.",
            "loop": True,
            "try_acquire_after_each_step": True,
            "steps": [
                {"type": "zoom", "delta_frames": -1, "name": "wide_back_1"},
                {"type": "ptz", "direction": "last", "pan_norm": 0.55, "tilt_norm": 0.0, "mode": "hold", "hold_ms": 500, "pulse_ms": 100, "name": "pan_last_short"},
                {"type": "pause", "sec": 0.35, "name": "pause"},
                {"type": "ptz", "direction": "opposite", "pan_norm": 0.55, "tilt_norm": 0.0, "mode": "hold", "hold_ms": 500, "pulse_ms": 100, "name": "pan_back_short"},
                {"type": "zoom", "delta_frames": 1, "name": "tele_forward_1"},
                {"type": "run_preset", "preset": "lost_step_wait", "name": "run_preset_1"},
                {"type": "pause", "sec": 5.0, "name": "pause_5s"}
            ]
        }
    }
}

if not preset_path.exists():
    preset_path.write_text(json.dumps(search_presets, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print("OK wrote ptz_search_presets.json")
else:
    old = json.loads(preset_path.read_text(encoding="utf-8"))
    changed = False
    old.setdefault("version", 1)
    old.setdefault("active", "lost_step_wait")
    old.setdefault("presets", {})
    for k, v in search_presets["presets"].items():
        if k not in old["presets"]:
            old["presets"][k] = v
            changed = True
    if changed:
        preset_path.write_text(json.dumps(old, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print("OK merged ptz_search_presets.json")
    else:
        print("OK ptz_search_presets.json already has defaults")

# 2. Patch apply_ptz_object_preset.py.
s = apply_path.read_text(encoding="utf-8", errors="ignore")
bak = apply_path.with_suffix(apply_path.suffix + f".bak_frame_search_presets_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

if 'SEARCH_PRESETS_FILE = ROOT / "ptz_search_presets.json"' not in s:
    anchor = 'PRESET_FILE = ROOT / "ptz_object_presets.json"\n'
    if anchor not in s:
        raise SystemExit("ERROR: PRESET_FILE anchor not found")
    s = s.replace(anchor, anchor + 'SEARCH_PRESETS_FILE = ROOT / "ptz_search_presets.json"\n', 1)

helper = r'''
def load_search_presets():
    default_data = {
        "version": 1,
        "active": "lost_step_wait",
        "presets": {}
    }

    base = load_json_file(SEARCH_PRESETS_FILE, default_data)
    settings = load_json_file(SETTINGS_FILE, {})

    presets = dict(base.get("presets") or {})

    custom = settings.get("searchPresetsCustom") or {}
    if isinstance(custom, dict):
        presets.update(custom)

    active = (
        settings.get("activeSearchPreset")
        or base.get("active")
        or "lost_step_wait"
    )

    if active not in presets and presets:
        active = sorted(presets.keys())[0]

    return base, settings, presets, active


def zoom_frame_state():
    try:
        st = get_json(f"{MJPEG_BASE}/api/zoom/state", timeout=3)
        sample = int(st.get("zoom_sample_idx") or 0)
        count = int(st.get("zoom_sample_count") or 10)
        ratio = float(st.get("zoom_ratio") or 0.0)
        count = max(1, count)
        sample = max(0, min(count - 1, sample))
        return {
            "ok": True,
            "raw": st,
            "sample": sample,
            "count": count,
            "ratio": ratio
        }
    except Exception as e:
        return {
            "ok": False,
            "error": str(e),
            "raw": {},
            "sample": 0,
            "count": 10,
            "ratio": 0.0
        }


def apply_ptz_speed_profile_for_zoom(sample_idx, zoom_ratio):
    try:
        return post_json(f"{AUTOPILOT_BASE}/api/autopilot/speed_profile/apply_nearest", {
            "profile_idx": int(sample_idx),
            "zoom_ratio": float(zoom_ratio)
        }, timeout=3)
    except Exception as e:
        event_log("ptz_speed_profile_apply_for_zoom_failed", error=str(e), sample_idx=sample_idx, zoom_ratio=zoom_ratio)
        return {"ok": False, "error": str(e)}


def zoom_move_frames(delta_frames, reason="search_preset"):
    """
    Frame-based zoom only.
    Uses /api/zoom/go_to_sample so zoom state and PTZ SPEED TUNE sample stay synchronized.
    """
    st = zoom_frame_state()
    cur = int(st.get("sample") or 0)
    count = int(st.get("count") or 10)
    count = max(1, count)

    delta = int(delta_frames or 0)
    target = max(0, min(count - 1, cur + delta))

    if target == cur:
        speed = apply_ptz_speed_profile_for_zoom(cur, st.get("ratio") or 0.0)
        event_log(
            "zoom_frame_skip_edge",
            reason=reason,
            current_sample=cur,
            target_sample=target,
            delta_frames=delta,
            sample_count=count,
            speed_profile=speed
        )
        return {
            "ok": True,
            "skipped": True,
            "reason": "edge_or_zero_delta",
            "current_sample": cur,
            "target_sample": target,
            "sample_count": count
        }

    try:
        res = post_json(f"{MJPEG_BASE}/api/zoom/go_to_sample", {
            "profile_idx": target,
            "mode": "sample",
            "source": reason
        }, timeout=25)

        st2 = zoom_frame_state()
        sample2 = int(st2.get("sample") or target)
        ratio2 = float(st2.get("ratio") or 0.0)

        speed = apply_ptz_speed_profile_for_zoom(sample2, ratio2)

        event_log(
            "zoom_frame_move",
            reason=reason,
            from_sample=cur,
            to_sample=target,
            actual_sample=sample2,
            delta_frames=delta,
            zoom_ratio=ratio2,
            response=res,
            speed_profile=speed
        )

        return {
            "ok": bool(res.get("ok", True)),
            "from_sample": cur,
            "to_sample": target,
            "actual_sample": sample2,
            "zoom_ratio": ratio2,
            "response": res,
            "speed_profile": speed
        }

    except Exception as e:
        event_log(
            "zoom_frame_move_failed",
            reason=reason,
            current_sample=cur,
            target_sample=target,
            delta_frames=delta,
            error=str(e)
        )
        return {"ok": False, "error": str(e), "current_sample": cur, "target_sample": target}


def target_direction_from_last(last_target, default=1):
    """
    Returns +1 right, -1 left.
    Uses last known target center if available.
    """
    try:
        center = item_center_norm(last_target or {})
        if center:
            cx, cy = center
            return 1 if float(cx) >= 0.5 else -1
    except Exception:
        pass

    return 1 if int(default or 1) >= 0 else -1


def ptz_hold_by_pulses(pan_norm, tilt_norm=0.0, hold_ms=500, pulse_ms=100, source="search_preset"):
    """
    /api/control/manual_j_pulse is short-pulse oriented.
    Long HOLD is emulated as repeated short pulses.
    """
    hold_ms = max(1, int(hold_ms or 1))
    pulse_ms = max(20, min(120, int(pulse_ms or 100)))

    deadline = time.time() + hold_ms / 1000.0
    count = 0
    last = None

    while time.time() < deadline:
        last = manual_search_pulse(
            pan=float(pan_norm or 0.0),
            tilt=float(tilt_norm or 0.0),
            pulse_ms=pulse_ms,
            source=source
        )
        count += 1

    event_log(
        "ptz_hold_by_pulses",
        source=source,
        pan_norm=pan_norm,
        tilt_norm=tilt_norm,
        hold_ms=hold_ms,
        pulse_ms=pulse_ms,
        pulses=count,
        last=last
    )

    return {
        "ok": True,
        "pan_norm": pan_norm,
        "tilt_norm": tilt_norm,
        "hold_ms": hold_ms,
        "pulse_ms": pulse_ms,
        "pulses": count,
        "last": last
    }


def acquire_with_attempts(classes, attempts=8, delay_sec=0.25):
    last = None

    for _ in range(max(1, int(attempts or 1))):
        last = try_acquire_once(classes, tracker_wait_sec=4.5)
        if last.get("ok"):
            return last
        time.sleep(float(delay_sec or 0.25))

    return last or {"ok": False, "reason": "no_attempts"}


def run_named_search_preset_once(name, classes, object_preset_name, last_target=None, depth=0, reason="lost"):
    base, settings, presets, active = load_search_presets()

    if not name:
        name = active

    if name not in presets:
        event_log("search_preset_not_found", preset=name, available=sorted(presets.keys()))
        return {"ok": False, "reason": "search_preset_not_found", "preset": name}

    if depth > 3:
        return {"ok": False, "reason": "search_preset_recursion_limit", "preset": name}

    preset = presets[name]
    steps = list(preset.get("steps") or [])
    loop = bool(preset.get("loop") is True)
    try_after_each = bool(preset.get("try_acquire_after_each_step") is True)

    default_dir = target_direction_from_last(last_target, default=1)

    event_log(
        "search_preset_start",
        search_preset=name,
        label=preset.get("label") or name,
        object_preset=object_preset_name,
        reason=reason,
        loop=loop,
        steps=len(steps),
        direction=default_dir
    )

    cycle = 0

    while True:
        for idx, step in enumerate(steps):
            typ = str(step.get("type") or "").strip()
            step_name = step.get("name") or f"step_{idx}"

            event_log(
                "search_preset_step",
                search_preset=name,
                step=step_name,
                type=typ,
                cycle=cycle,
                raw=step
            )

            if typ == "zoom":
                zoom_move_frames(
                    int(step.get("delta_frames") or 0),
                    reason=f"search_preset_{name}_{step_name}"
                )

            elif typ == "ptz":
                direction_name = str(step.get("direction") or "last")
                direction = default_dir

                if direction_name == "opposite":
                    direction = -default_dir
                elif direction_name == "right":
                    direction = 1
                elif direction_name == "left":
                    direction = -1

                pan_norm = float(step.get("pan_norm") or 0.0) * direction
                tilt_norm = float(step.get("tilt_norm") or 0.0)

                ptz_hold_by_pulses(
                    pan_norm=pan_norm,
                    tilt_norm=tilt_norm,
                    hold_ms=int(step.get("hold_ms") or 500),
                    pulse_ms=int(step.get("pulse_ms") or 100),
                    source=f"search_preset_{name}_{step_name}"
                )

            elif typ == "pause":
                sec = max(0.0, float(step.get("sec") or 0.0))
                time.sleep(sec)

            elif typ == "acquire":
                res = acquire_with_attempts(
                    classes,
                    attempts=int(step.get("attempts") or 8),
                    delay_sec=float(step.get("delay_sec") or 0.25)
                )
                if res.get("ok"):
                    event_log("search_preset_acquired", search_preset=name, step=step_name, result=res)
                    return {"ok": True, "preset": name, "result": res}

            elif typ == "run_preset":
                sub = str(step.get("preset") or "")
                res = run_named_search_preset_once(
                    sub,
                    classes,
                    object_preset_name,
                    last_target=last_target,
                    depth=depth + 1,
                    reason=f"nested_from_{name}"
                )
                if res.get("ok"):
                    return res

            if try_after_each and typ not in {"pause", "acquire"}:
                res = acquire_with_attempts(classes, attempts=2, delay_sec=0.15)
                if res.get("ok"):
                    event_log("search_preset_acquired_after_step", search_preset=name, step=step_name, result=res)
                    return {"ok": True, "preset": name, "result": res}

        cycle += 1

        if not loop:
            break

        # In watch mode loop is okay, but keep each preset cycle bounded enough
        # so outer state machine can re-check settings/daemon state.
        if cycle >= 1:
            break

    event_log("search_preset_finished_no_target", search_preset=name, cycle=cycle)
    return {"ok": False, "preset": name, "reason": "target_not_acquired"}


def run_active_search_preset_once(classes, object_preset_name, last_target=None, reason="lost"):
    base, settings, presets, active = load_search_presets()
    return run_named_search_preset_once(
        active,
        classes,
        object_preset_name,
        last_target=last_target,
        depth=0,
        reason=reason
    )

'''

if "def load_search_presets()" not in s:
    anchor = "\ndef run_once_fail_stop("
    if anchor not in s:
        raise SystemExit("ERROR: run_once_fail_stop anchor not found")
    s = s.replace(anchor, "\n" + helper + anchor, 1)

start = s.find("def run_continuous_wide_scan_x(")
if start < 0:
    raise SystemExit("ERROR: run_continuous_wide_scan_x not found")

end = s.find("\ndef run_custom_search_plan(", start)
if end < 0:
    raise SystemExit("ERROR: run_custom_search_plan marker not found")

new_func = r'''def run_continuous_wide_scan_x(classes, preset_name, plan, max_seconds=0):
    stop_ptz()

    lost_seen_since = None
    last_status = None
    last_target = None

    lost_grace_sec = float(plan.get("lost_grace_sec") or 1.8)
    lost_grace_poll_sec = float(plan.get("lost_grace_poll_sec") or 0.25)

    deadline = None
    if max_seconds and float(max_seconds) > 0:
        deadline = time.time() + float(max_seconds)

    event_log("continuous_frame_search_start", preset=preset_name, classes=classes)

    while True:
        if deadline and time.time() >= deadline:
            stop_ptz()
            out = {
                "ok": False,
                "mode": "continuous_wide_scan_x",
                "reason": "max_seconds_reached",
                "preset": preset_name
            }
            event_log("continuous_frame_search_finished", **out)
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

                if isinstance(tr.get("box"), dict):
                    last_target = tr.get("box")

                status = {
                    "mode": "tracking",
                    "preset": preset_name,
                    "track_id": tr.get("selected_track_id"),
                    "ptz": ap.get("mode"),
                    "cmd_pan": ap.get("cmd_pan"),
                    "cmd_tilt": ap.get("cmd_tilt"),
                    "zoom_sample": ap.get("active_zoom_sample_idx"),
                    "zoom_ratio": ap.get("zoom_ratio")
                }

                if status != last_status:
                    event_log("tracking", **status)
                    write_state(**status)
                    last_status = status

                time.sleep(0.5)
                continue

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
                    "lost_object_frame_search",
                    preset=preset_name,
                    lost_age=round(lost_age, 3),
                    tracker=tr,
                    autopilot=ap,
                    last_target=last_target
                )

                stop_ptz()
                lost_seen_since = None

                search_res = run_active_search_preset_once(
                    classes,
                    preset_name,
                    last_target=last_target,
                    reason="lost_object"
                )

                if search_res.get("ok"):
                    write_state(mode="tracking", preset=preset_name, result=search_res)
                    continue

            acquire = try_acquire_once(classes, tracker_wait_sec=float(plan.get("tracker_wait_sec") or 4.5))

            if acquire.get("ok"):
                write_state(mode="tracking", preset=preset_name, result=acquire)
                continue

            search_res = run_active_search_preset_once(
                classes,
                preset_name,
                last_target=last_target,
                reason="idle_search"
            )

            if search_res.get("ok"):
                write_state(mode="tracking", preset=preset_name, result=search_res)
                continue

            write_state(
                mode="searching",
                preset=preset_name,
                active_search_preset=load_search_presets()[3],
                result=search_res
            )

            time.sleep(0.3)

        except KeyboardInterrupt:
            stop_ptz()
            out = {"ok": False, "mode": "continuous_wide_scan_x", "reason": "interrupted"}
            event_log("continuous_frame_search_interrupted", preset=preset_name)
            write_state(mode="stopped", preset=preset_name, result=out)
            return out

        except Exception as e:
            event_log("continuous_frame_search_exception", preset=preset_name, error=str(e))
            write_state(mode="search_error", preset=preset_name, error=str(e))
            time.sleep(0.5)

'''

s = s[:start] + new_func + s[end:]

# Extend --list output with search preset info.
old = '''        for name, preset in presets.items():
            print(
                name,
                "=",
                preset.get("label") or name,
                "|",
                preset.get("tracking_mode") or "manual_select",
                "|",
                preset.get("loss_behavior") or "once_fail_stop"
            )
        return 0
'''

new = '''        for name, preset in presets.items():
            print(
                name,
                "=",
                preset.get("label") or name,
                "|",
                preset.get("tracking_mode") or "manual_select",
                "|",
                preset.get("loss_behavior") or "once_fail_stop"
            )

        try:
            sb, ss, sp, sa = load_search_presets()
            print("")
            print("activeSearchPreset =", sa)
            for sname, spreset in sp.items():
                print("search", sname, "=", spreset.get("label") or sname)
        except Exception as e:
            print("search presets error =", e)

        return 0
'''

if old in s and "activeSearchPreset =" not in s:
    s = s.replace(old, new, 1)

apply_path.write_text(s, encoding="utf-8")
print("OK patched apply_ptz_object_preset.py")
print("Backup:", bak)
