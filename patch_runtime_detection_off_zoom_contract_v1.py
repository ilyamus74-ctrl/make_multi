#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
path = ROOT / 'apply_ptz_object_preset.py'
text = path.read_text(encoding='utf-8', errors='ignore')
orig = text

if 'PTZ_RUNTIME_DETECTION_OFF_PAUSE_GUARD_V1' in text:
    print('Already patched: PTZ_RUNTIME_DETECTION_OFF_PAUSE_GUARD_V1 present')
else:
    backup = path.with_name(path.name + f'.bak_detection_off_zoom_contract_v1_{int(time.time())}')
    backup.write_text(text, encoding='utf-8')

    # 1) Insert detector-off guard helpers after zoom_frame_state() block, before apply_ptz_speed_profile_for_zoom.
    marker = '\n\ndef apply_ptz_speed_profile_for_zoom(sample_idx, zoom_ratio):\n'
    if marker not in text:
        raise SystemExit('Could not find apply_ptz_speed_profile_for_zoom insertion point')

    helpers = r'''

# PTZ_RUNTIME_DETECTION_OFF_PAUSE_GUARD_V1_START
def detector_runtime_enabled_state():
    """
    Runtime guard source: detector config.
    Returns (enabled, raw). API errors are treated as enabled to avoid false emergency pauses.
    """
    try:
        raw = get_json(f"{MJPEG_BASE}/api/detector/config", timeout=2)
        if isinstance(raw, dict) and raw.get("detect_enabled") is False:
            return False, raw
        return True, raw
    except Exception as e:
        return True, {"ok": False, "error": str(e)}


def runtime_pause_if_detection_disabled(context="runtime", preset_name=None):
    """
    If operator disables Detection while PTZ ACTIVE, runtime must not search/reacquire.
    It stops PTZ and pauses until detection is enabled again.
    """
    enabled, raw = detector_runtime_enabled_state()
    if enabled is not False:
        return False

    try:
        stop_ptz()
    except Exception as e:
        event_log("runtime_detection_disabled_stop_ptz_failed", context=context, preset=preset_name, error=str(e))

    event_log(
        "runtime_detection_disabled_pause",
        context=context,
        preset=preset_name,
        detector=raw
    )
    try:
        write_state(
            mode="paused_detection_off",
            preset=preset_name,
            context=context,
            detector=raw
        )
    except Exception:
        pass
    return True
# PTZ_RUNTIME_DETECTION_OFF_PAUSE_GUARD_V1_END
'''
    text = text.replace(marker, helpers + marker, 1)

    # 2) Replace zoom_wide_pulse raw jog with sample/profile contract implementation.
    start = text.find('\ndef zoom_wide_pulse(')
    end = text.find('\ndef manual_search_pulse(', start)
    if start == -1 or end == -1:
        raise SystemExit('Could not find zoom_wide_pulse block boundaries')

    new_zoom_wide = r'''
def zoom_wide_pulse(reason="search", hold_ms=220, cooldown_sec=8.0, skip_ratio=0.06):
    """
    PTZ_RUNTIME_ZOOM_SAMPLE_SPEED_SYNC_V1
    Wide zoom movement must use the sample contract:
      /api/zoom/go_to_sample {profile_idx}
      /api/autopilot/speed_profile/apply_nearest {profile_idx}
    Never raw /api/zoom/jog here; raw jog can desync physical zoom and PTZ SPEED TUNE sample.
    """
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
        st = zoom_frame_state()
        cur = int(st.get("sample") or 0)
        count = max(1, int(st.get("count") or 10))
        target = 0

        res = post_json(f"{MJPEG_BASE}/api/zoom/go_to_sample", {
            "profile_idx": target,
            "mode": "sample",
            "source": f"auto_arm_{reason}"
        }, timeout=25)

        st2 = zoom_frame_state()
        sample2 = int(st2.get("sample") or target)
        ratio2 = float(st2.get("ratio") or 0.0)
        speed = apply_ptz_speed_profile_for_zoom(sample2, ratio2)

        zoom_wide_pulse._last_ts = now
        event_log(
            "zoom_wide_sample_move",
            reason=reason,
            ok=True,
            current_sample=cur,
            target_sample=target,
            final_sample=sample2,
            sample_count=count,
            zoom_ratio=ratio2,
            go_to_sample=res,
            speed_profile=speed
        )
        return {
            "ok": True,
            "current_sample": cur,
            "target_sample": target,
            "final_sample": sample2,
            "sample_count": count,
            "zoom_ratio": ratio2,
            "go_to_sample": res,
            "speed_profile": speed
        }

    except Exception as e:
        event_log("zoom_wide_sample_move", reason=reason, ok=False, error=str(e))
        return {"ok": False, "error": str(e)}

'''
    text = text[:start] + new_zoom_wide + text[end:]

    # 3) Add detection-off pause in search preset step loop.
    old = '''    while True:\n        for idx, step in enumerate(steps):\n'''
    new = '''    while True:\n        if runtime_pause_if_detection_disabled("search_preset", object_preset_name):\n            return {"ok": False, "preset": name, "reason": "detection_disabled", "search_preset": name}\n\n        for idx, step in enumerate(steps):\n            if runtime_pause_if_detection_disabled("search_preset_step", object_preset_name):\n                return {"ok": False, "preset": name, "reason": "detection_disabled", "search_preset": name}\n'''
    if old not in text:
        raise SystemExit('Could not patch run_named_search_preset_once while loop')
    text = text.replace(old, new, 1)

    # 4) Add detection-off pause in continuous wide scan loop.
    old = '''    while True:\n        if deadline and time.time() >= deadline:\n'''
    new = '''    while True:\n        if runtime_pause_if_detection_disabled("continuous_wide_scan_x", preset_name):\n            lost_seen_since = None\n            last_status = None\n            time.sleep(0.5)\n            continue\n\n        if deadline and time.time() >= deadline:\n'''
    if old not in text:
        raise SystemExit('Could not patch run_continuous_wide_scan_x while loop')
    text = text.replace(old, new, 1)

    # 5) Add detection-off pause in custom plan loop (second while True after custom_plan_start).
    old = '''    while True:\n        if deadline and time.time() >= deadline:\n'''
    new = '''    while True:\n        if runtime_pause_if_detection_disabled("custom_plan", preset_name):\n            time.sleep(0.5)\n            continue\n\n        if deadline and time.time() >= deadline:\n'''
    if old not in text:
        raise SystemExit('Could not patch custom plan while loop')
    text = text.replace(old, new, 1)

    path.write_text(text, encoding='utf-8')
    print('OK patched apply_ptz_object_preset.py runtime detection-off + zoom sample contract v1')
    print('Backup:', backup)
    print('Changes:')
    print(' - added runtime detection-off pause guard')
    print(' - search/reacquire pauses when detect_enabled=false')
    print(' - replaced zoom_wide_pulse raw /api/zoom/jog with /api/zoom/go_to_sample profile_idx=0')
    print(' - applies PTZ speed profile after wide zoom sample move')

# marker verification
s = path.read_text(encoding='utf-8', errors='ignore')
checks = [
    'PTZ_RUNTIME_DETECTION_OFF_PAUSE_GUARD_V1_START',
    'PTZ_RUNTIME_DETECTION_OFF_PAUSE_GUARD_V1_END',
    'PTZ_RUNTIME_ZOOM_SAMPLE_SPEED_SYNC_V1',
    'runtime_detection_disabled_pause',
    'runtime_pause_if_detection_disabled("search_preset"',
    'runtime_pause_if_detection_disabled("continuous_wide_scan_x"',
    'runtime_pause_if_detection_disabled("custom_plan"',
    'zoom_wide_sample_move',
]
print('')
print('Marker counts:')
for c in checks:
    print(f'{c} = {s.count(c)}')

# scoped check for zoom_wide_pulse raw jog
start = s.find('def zoom_wide_pulse(')
end = s.find('def manual_search_pulse(', start)
block = s[start:end] if start != -1 and end != -1 else ''
print('')
print('zoom_wide_pulse scoped /api/zoom/jog =', block.count('/api/zoom/jog'))
print('zoom_wide_pulse scoped /api/zoom/go_to_sample =', block.count('/api/zoom/go_to_sample'))
print('zoom_wide_pulse scoped apply_ptz_speed_profile_for_zoom =', block.count('apply_ptz_speed_profile_for_zoom'))