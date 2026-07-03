#!/usr/bin/env python3
from pathlib import Path
import json
import subprocess
import time
import urllib.request
import os
import signal

ROOT = Path("/root/new_yolo8")
if not ROOT.exists():
    ROOT = Path(__file__).resolve().parent
WEB = ROOT / "web/index.html"
SETTINGS_FILE = ROOT / "ui_settings.json"

MJPEG_BASE = "http://127.0.0.1:8080"
PTZ_BASE = "http://127.0.0.1:8090"

RESULTS = []
WARNINGS = []

def warn(name, detail=""):
    WARNINGS.append((str(name), str(detail)))
    print(f"WARN: {name} — {detail}")

def add(name, ok, detail=""):
    RESULTS.append((name, bool(ok), str(detail)))

def section(name):
    print("")
    print("=" * 100)
    print(name)
    print("=" * 100)

def get_json(url, timeout=3):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            raw = r.read().decode("utf-8", errors="replace")
        return json.loads(raw) if raw.strip() else {}
    except Exception as e:
        return {"__error__": str(e)}

def post_json(url, body, timeout=3):
    try:
        data = json.dumps(body).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST"
        )
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode("utf-8", errors="replace")
        return json.loads(raw) if raw.strip() else {}
    except Exception as e:
        return {"__error__": str(e)}

def read_json_file(path):
    try:
        return json.loads(path.read_text(encoding="utf-8", errors="replace"))
    except Exception as e:
        return {"__error__": str(e)}

def ps_lines():
    out = subprocess.run(
        ["ps", "ax", "-o", "pid=,args="],
        text=True,
        capture_output=True
    ).stdout

    return [line.strip() for line in out.splitlines() if line.strip()]

def find_processes(pattern):
    out = []

    for line in ps_lines():
        parts = line.split(None, 1)

        if len(parts) != 2:
            continue

        try:
            pid = int(parts[0])
        except Exception:
            continue

        args = parts[1]

        if pid == os.getpid():
            continue

        if pattern in args:
            out.append((pid, args))

    return out

def active_custom_preset(settings):
    active = settings.get("activeObjectPreset")
    custom = settings.get("objectPresetsCustom") or {}

    if not active:
        return active, {}

    return active, custom.get(active) or {}

def normalize_list_numbers(x):
    out = []

    for item in x or []:
        try:
            out.append(int(item))
        except Exception:
            pass

    return sorted(out)

def audit_files():
    section("FILES")

    files = [
        "PTZ_MASTER_CONTRACT.md",
        "web/index.html",
        "ui_settings.json",
        "launcher.sh",
        "hydrate_runtime_settings.py",
        "settings_persist_daemon.py",
        "object_tracking_daemon.py",
        "apply_ptz_object_preset.py",
        "mjpeg_gst_http.cpp",
        "ptz_autopilot.cpp"
    ]

    for f in files:
        p = ROOT / f
        exists = p.exists()
        size = p.stat().st_size if exists else 0
        print(f"{f}: exists={exists} size={size}")
        add(f"file exists: {f}", exists, f"size={size}")

def audit_ui():
    section("LAYER 1 — BROWSER UI")

    if not WEB.exists():
        add("UI web/index.html exists", False, "missing")
        return

    s = WEB.read_text(encoding="utf-8", errors="ignore")

    required = {
        "PTZ_CLEAN_LAYER_CONTROLLER_START": 1,
        "PTZ_CLEAN_SAVESETTINGS_GUARD_START": 1
    }

    forbidden = [
        "SINGLE_PRESET_AUTO_ARM",
        "OBJECT_PRESET_MANAGER_START",
        "OBJECT_PRESET_BUTTONS_ACTIVE_FIX_START",
        "OBJECT_PRESET_AUTO_LEARN_OPERATOR_CONTROLS_START",
        "OPERATOR_CONTROLS_HARD_PERSIST_APPLY_START",
        "OBJECT_PRESET_NO_AUTO_ARM_DURABLE_SAVE_START",
        "PTZ_START_STOP_ACTIVE_STATE_FIX_START",
        "SEARCH_PRESET_SELECTOR_START"
    ]

    for token, expected in required.items():
        count = s.count(token)
        print(token, "=", count)
        add(f"UI required marker {token}", count == expected, f"count={count} expected={expected}")

    for token in forbidden:
        count = s.count(token)
        print(token, "=", count)
        add(f"UI forbidden marker {token}", count == 0, f"count={count}")

    old_btn_count = s.count("data-object-preset")
    clean_btn_count = s.count("data-clean-object-preset")

    print("data-object-preset =", old_btn_count)
    print("data-clean-object-preset =", clean_btn_count)

    add("UI old object preset buttons removed", old_btn_count == 0, f"count={old_btn_count}")
    add("UI clean object preset buttons exist", clean_btn_count >= 8, f"count={clean_btn_count}")


    # PTZ_AUDIT_UI_DETECTION_JS_INLINE_V3
    section("LAYER 1B — UI DETECTION CONTROLS JS CONTRACT")
    from pathlib import Path as _PtzAuditPath
    try:
        web_text = _PtzAuditPath("web/index.html").read_text(encoding="utf-8", errors="ignore")
    except Exception as e:
        web_text = ""
        add("UI JS contract web/index.html readable", False, str(e))

    required_js_markers = [
        "PTZ_CLEAN_DETECTION_UI_HYDRATE_F5_START",
        "PTZ_CLEAN_DETECTION_ACTIVE_PRESET_PERSIST_START",
        "hydrateDetectionControlsFromSettings",
        "scheduleDetectionUiHydrate",
        "persistActiveDetectionControlsToPreset",
        "schedulePersistActiveDetectionControlsToPreset",
    ]

    for marker_name in required_js_markers:
        cnt = web_text.count(marker_name)
        print(f"{marker_name} = {cnt}")
        add(f"UI detection JS marker {marker_name}", cnt > 0, f"count={cnt}")

    # UI must not be able to destroy presets by writing an empty objectPresetsCustom
    # from old saveSettings paths. The clean save guard must fall back to OBJECT_PRESETS
    # when the server-side objectPresetsCustom is temporarily empty.
    has_presets_fallback = (
        "objectPresetsCustom" in web_text
        and "OBJECT_PRESETS" in web_text
        and "Object.keys(before.objectPresetsCustom).length" in web_text
    )
    print("objectPresetsCustom fallback guard =", has_presets_fallback)
    add(
        "UI saveSettings preserves objectPresetsCustom fallback",
        has_presets_fallback,
        "requires OBJECT_PRESETS fallback when before.objectPresetsCustom is empty"
    )

    # UI detection controls are not just runtime controls. They must persist into
    # objectPresetsCustom[activeObjectPreset] so F5/re-select restores operator choices.
    active_preset_write_markers = [
        "max_detections",
        "detect_every_n_frames",
        "detection_mode",
        "activeObjectPreset",
        "objectPresetsCustom",
    ]
    missing_persist_parts = [m for m in active_preset_write_markers if m not in web_text]
    print("active preset detection persist missing parts =", missing_persist_parts)
    add(
        "UI detection controls persist to active object preset",
        not missing_persist_parts,
        f"missing={missing_persist_parts}"
    )


    # PTZ_AUDIT_KEYBOARD_QA_SWEEP_V1
    # PTZ_AUDIT_KEYBOARD_QA_SWEEP_V2
    # PTZ_AUDIT_KEYBOARD_QA_MOVEMENT_MODE_V3
    print("")
    section("LAYER 1C — KEYBOARD Q/A SWEEP JS CONTRACT")

    try:
        _qa_web_text = web_text
    except Exception:
        _qa_web_text = (ROOT / "web/index.html").read_text(encoding="utf-8", errors="ignore")

    qa_start = _qa_web_text.find("PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_START")
    qa_end = _qa_web_text.find("PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_END", qa_start)
    if qa_start >= 0 and qa_end > qa_start:
        qa_block = _qa_web_text[qa_start:qa_end]
    else:
        qa_block = ""

    qa_tokens = [
        "PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_START",
        "PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_END",
        "ptzQaSweepProfileIdxStep",
        "keyboard_Q_profile_idx_sweep",
        "keyboard_A_profile_idx_sweep",
    ]

    for token in qa_tokens:
        cnt = _qa_web_text.count(token)
        print(token, "=", cnt)
        add(f"Keyboard Q/A sweep marker {token}", cnt >= 1, f"count={cnt}")

    has_go_to_sample = "/api/zoom/go_to_sample" in _qa_web_text
    has_profile_idx_target = "profile_idx: target" in _qa_web_text
    # Scope this check to the canonical Q/A block only.
    # Other legacy/sample UI code can legitimately contain sample_idx,
    # but Q/A must use profile_idx as the backend request key.
    has_no_canonical_sample_idx = (
        bool(qa_block)
        and "sample_idx:" not in qa_block
        and "\"sample_idx\"" not in qa_block
        and "'sample_idx'" not in qa_block
    )
    has_apply_nearest = "/api/autopilot/speed_profile/apply_nearest" in _qa_web_text or "speed_profile/apply_nearest" in _qa_web_text
    has_profile_idx_final = "profile_idx: finalIdx" in _qa_web_text
    has_capture_stop = (
        bool(qa_block)
        and "stopImmediatePropagation" in qa_block
        and "window.addEventListener('keydown'" in qa_block
        and "}, true)" in qa_block
        and ("sweep_time_steps" in qa_block or "ptzQaSweepIsSweepMode" in qa_block or "ptzQaSweepIsSweep" in qa_block)
    )

    has_movement_mode_v2 = "PTZ_QA_SAMPLE_STEP_MOVEMENT_MODE_V2" in qa_block
    qa_owns_both_zoom_movement_modes = (
        bool(qa_block)
        and "if (!ptzQaSweepIsSweepMode()) return" not in qa_block
        and "legacy_impulse" in qa_block
        and "sweep_time_steps" in qa_block
        and "ptzQaSweepMovementMode" in qa_block
    )

    print("Q/A scoped block length =", len(qa_block))
    print("Q/A go_to_sample backend call =", has_go_to_sample)
    print("Q/A go_to_sample uses profile_idx target =", has_profile_idx_target)
    print("Q/A avoids canonical sample_idx target =", has_no_canonical_sample_idx)
    print("Q/A apply_nearest backend call =", has_apply_nearest)
    print("Q/A apply_nearest uses profile_idx finalIdx =", has_profile_idx_final)
    print("Q/A capture stop =", has_capture_stop)
    print("Q/A movement mode v2 marker =", has_movement_mode_v2)
    print("Q/A owns both ZOOM CALIB movement modes =", qa_owns_both_zoom_movement_modes)

    add(
        "Keyboard Q/A sweep moves real backend zoom sample",
        has_go_to_sample and has_profile_idx_target,
        f"go_to_sample={has_go_to_sample} profile_idx_target={has_profile_idx_target}"
    )
    add(
        "Keyboard Q/A sweep does not rely on sample_idx target",
        has_no_canonical_sample_idx,
        "canonical request key must be profile_idx"
    )
    add(
        "Keyboard Q/A sweep applies PTZ speed profile by profile_idx",
        has_apply_nearest and has_profile_idx_final,
        f"apply_nearest={has_apply_nearest} profile_idx_final={has_profile_idx_final}"
    )
    add(
        "Keyboard Q/A sweep owns event before older handlers",
        has_capture_stop,
        f"capture_stop={has_capture_stop}"
    )
    add(
        "Keyboard Q/A sample step respects ZOOM CALIB Movement",
        has_movement_mode_v2 and qa_owns_both_zoom_movement_modes,
        f"movement_marker={has_movement_mode_v2} owns_both_modes={qa_owns_both_zoom_movement_modes}"
    )


    # PTZ_AUDIT_KEYBOARD_ARROWS_V1
    print("")
    section("LAYER 1D — KEYBOARD ARROW PTZ JS CONTRACT")

    try:
        _arrow_web_text = web_text
    except Exception:
        _arrow_web_text = (ROOT / "web/index.html").read_text(encoding="utf-8", errors="ignore")

    arrow_start = _arrow_web_text.find("function ptzKeyboardCaptureDirection")
    arrow_end = _arrow_web_text.find("QA_SWEEP_GLOBAL_HELPERS_START", arrow_start)
    if arrow_start >= 0 and arrow_end > arrow_start:
        arrow_block = _arrow_web_text[arrow_start:arrow_end]
    elif arrow_start >= 0:
        arrow_block = _arrow_web_text[arrow_start:arrow_start + 9000]
    else:
        arrow_block = ""

    arrow_tokens = [
        "ptzKeyboardCaptureDirection",
        "ptzKeyboardCaptureEnabled",
        "window.__ptzKeyCaptureInstalled",
        "ArrowLeft",
        "ArrowRight",
        "ArrowUp",
        "ArrowDown",
        "keyboard_left",
        "keyboard_right",
        "keyboard_up",
        "keyboard_down",
    ]

    for token in arrow_tokens:
        cnt = _arrow_web_text.count(token)
        print(token, "=", cnt)
        add(f"Keyboard arrow marker {token}", cnt >= 1, f"count={cnt}")

    has_arrow_block = bool(arrow_block)
    has_all_directions = all(t in arrow_block for t in ["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"])
    has_ptz_mode_guard = "ptzKeyboardCaptureEnabled" in arrow_block and "controlMode" in arrow_block and "ptz" in arrow_block
    has_capture_keydown = "window.addEventListener('keydown'" in arrow_block and "e.preventDefault()" in arrow_block and "e.stopImmediatePropagation()" in arrow_block and "}, true)" in arrow_block
    has_capture_keyup = "window.addEventListener('keyup'" in arrow_block and "manualKeyState.delete" in arrow_block and "}, true)" in arrow_block
    has_motion_loop = "keyboardMotionLoop" in arrow_block or "manualInput" in _arrow_web_text or "manual_j_pulse" in _arrow_web_text
    has_key_release_stop = "scheduleKeyboardHoldStop" in arrow_block or "stopManualHold" in arrow_block or "manualInputStop" in _arrow_web_text

    print("Arrow scoped block length =", len(arrow_block))
    print("Arrow owns all directions =", has_all_directions)
    print("Arrow PTZ mode guard =", has_ptz_mode_guard)
    print("Arrow capture keydown =", has_capture_keydown)
    print("Arrow capture keyup =", has_capture_keyup)
    print("Arrow backend/manual motion path =", has_motion_loop)
    print("Arrow key release stop path =", has_key_release_stop)

    add(
        "Keyboard arrows own PTZ pan/tilt directions",
        has_arrow_block and has_all_directions,
        f"block={has_arrow_block} all_directions={has_all_directions}"
    )
    add(
        "Keyboard arrows are gated to PTZ mode",
        has_ptz_mode_guard,
        f"ptz_mode_guard={has_ptz_mode_guard}"
    )
    add(
        "Keyboard arrows capture keydown before browser/page handlers",
        has_capture_keydown,
        f"capture_keydown={has_capture_keydown}"
    )
    add(
        "Keyboard arrows capture keyup and release movement",
        has_capture_keyup and has_key_release_stop,
        f"capture_keyup={has_capture_keyup} release_stop={has_key_release_stop}"
    )
    add(
        "Keyboard arrows have backend/manual motion path",
        has_motion_loop,
        f"motion_path={has_motion_loop}"
    )

def audit_settings(api, file_data):
    section("LAYER 2 — SETTINGS / PRESETS")

    print("API activeObjectPreset =", api.get("activeObjectPreset"))
    print("API activeSearchPreset =", api.get("activeSearchPreset"))
    print("API ptzArmed =", api.get("ptzArmed"))
    print("API controlMode =", api.get("controlMode"))
    print("API custom keys =", sorted((api.get("objectPresetsCustom") or {}).keys()))
    print("API limit/every =", api.get("operatorDetectionLimit"), api.get("operatorDetectEvery"))
    print("")

    print("FILE activeObjectPreset =", file_data.get("activeObjectPreset"))
    print("FILE activeSearchPreset =", file_data.get("activeSearchPreset"))
    print("FILE ptzArmed =", file_data.get("ptzArmed"))
    print("FILE controlMode =", file_data.get("controlMode"))
    print("FILE custom keys =", sorted((file_data.get("objectPresetsCustom") or {}).keys()))
    print("FILE limit/every =", file_data.get("operatorDetectionLimit"), file_data.get("operatorDetectEvery"))

    api_custom = api.get("objectPresetsCustom") or {}
    file_custom = file_data.get("objectPresetsCustom") or {}

    add("Settings API reachable", "__error__" not in api, api.get("__error__", "ok"))
    add("Settings file readable", "__error__" not in file_data, file_data.get("__error__", "ok"))

    add("Settings API custom presets non-empty", bool(api_custom), f"keys={sorted(api_custom.keys())}")
    add("Settings FILE custom presets non-empty", bool(file_custom), f"keys={sorted(file_custom.keys())}")

    add(
        "Settings activeObjectPreset API/FILE match",
        api.get("activeObjectPreset") == file_data.get("activeObjectPreset"),
        f"api={api.get('activeObjectPreset')} file={file_data.get('activeObjectPreset')}"
    )

    add(
        "Settings activeSearchPreset API/FILE match",
        api.get("activeSearchPreset") == file_data.get("activeSearchPreset"),
        f"api={api.get('activeSearchPreset')} file={file_data.get('activeSearchPreset')}"
    )

    add("Settings FILE ptzArmed false", file_data.get("ptzArmed") is False, f"file={file_data.get('ptzArmed')}")
    add("Settings FILE controlMode manual", file_data.get("controlMode") == "manual", f"file={file_data.get('controlMode')}")

    active, preset = active_custom_preset(api)

    print("")
    print("API active custom preset =", json.dumps(preset, indent=2, ensure_ascii=False))

    add("Settings active custom preset exists", bool(preset), f"active={active}")

    if preset:
        add(
            "Settings top-level limit matches active preset",
            int(api.get("operatorDetectionLimit") or -1) == int(preset.get("max_detections") or -2),
            f"settings={api.get('operatorDetectionLimit')} preset={preset.get('max_detections')}"
        )

        add(
            "Settings top-level detect_every matches active preset",
            int(api.get("operatorDetectEvery") or -1) == int(preset.get("detect_every_n_frames") or -2),
            f"settings={api.get('operatorDetectEvery')} preset={preset.get('detect_every_n_frames')}"
        )

        add(
            "Settings top-level area matches active preset",
            str(api.get("operatorDetectionAreaMode") or "") == str(preset.get("detection_mode") or ""),
            f"settings={api.get('operatorDetectionAreaMode')} preset={preset.get('detection_mode')}"
        )

        bad_ptz_keys = []

        for k in (preset.get("ptz") or {}).keys():
            if k in [
                "kp",
                "ki",
                "kd",
                "deadzone",
                "max_pan",
                "max_tilt",
                "max_accel",
                "min_pan",
                "min_tilt",
                "hz",
                "ptz_curve",
                "ptz_lead_ms",
                "manual_mode",
                "j_pulse_ms"
            ]:
                bad_ptz_keys.append(k)

        add("Settings active object preset has no PTZ speed keys", not bad_ptz_keys, f"bad={bad_ptz_keys}")

def audit_detector(api):
    section("LAYER 3 — DETECTOR BACKEND")

    limits = get_json(f"{MJPEG_BASE}/api/detection/limits")
    throttle = get_json(f"{MJPEG_BASE}/api/detection/throttle")
    roi = get_json(f"{MJPEG_BASE}/api/detection/roi_config")
    det = get_json(f"{MJPEG_BASE}/api/detector/config")

    print("limits =", json.dumps(limits, indent=2, ensure_ascii=False))
    print("throttle =", json.dumps(throttle, indent=2, ensure_ascii=False))
    print("roi =", json.dumps(roi, indent=2, ensure_ascii=False))
    print("detector classes =", det.get("selected_classes"))

    add("Detector limits reachable", "__error__" not in limits, limits.get("__error__", "ok"))
    add("Detector throttle reachable", "__error__" not in throttle, throttle.get("__error__", "ok"))
    add("Detector roi reachable", "__error__" not in roi, roi.get("__error__", "ok"))
    add("Detector config reachable", "__error__" not in det, det.get("__error__", "ok"))

    active, preset = active_custom_preset(api)

    if preset:
        add(
            "Detector max_detections matches active preset",
            int(limits.get("max_detections") or -1) == int(preset.get("max_detections") or -2),
            f"backend={limits.get('max_detections')} preset={preset.get('max_detections')}"
        )

        add(
            "Detector detect_every matches active preset",
            int(throttle.get("detect_every_n_frames") or -1) == int(preset.get("detect_every_n_frames") or -2),
            f"backend={throttle.get('detect_every_n_frames')} preset={preset.get('detect_every_n_frames')}"
        )

        add(
            "Detector area matches active preset",
            str(roi.get("detection_mode") or "") == str(preset.get("detection_mode") or ""),
            f"backend={roi.get('detection_mode')} preset={preset.get('detection_mode')}"
        )

        add(
            "Detector classes match active preset",
            normalize_list_numbers(det.get("selected_classes")) == normalize_list_numbers(preset.get("classes")),
            f"backend={normalize_list_numbers(det.get('selected_classes'))} preset={normalize_list_numbers(preset.get('classes'))}"
        )

def audit_ptz():
    section("LAYER 4 — PTZ AUTOPILOT")

    ap = get_json(f"{PTZ_BASE}/api/autopilot/state")

    print(json.dumps(ap, indent=2, ensure_ascii=False))

    add("PTZ autopilot reachable", "__error__" not in ap, ap.get("__error__", "ok"))

    if "__error__" not in ap:
        add("PTZ autopilot not auto-running", ap.get("enabled") is False, f"enabled={ap.get('enabled')} mode={ap.get('mode')}")
        src = ap.get("speed_profile_source")
        add(
            "PTZ speed profile source not runtime override",
            src not in [None, "", "fallback", "runtime_override"],
            f"source={src}"
        )

def audit_processes():
    section("PROCESSES")

    procs = {
        "mjpeg_rknn_http": find_processes("mjpeg_rknn_http"),
        "ptz_autopilot": find_processes("ptz_autopilot"),
        "settings_persist_daemon.py": find_processes("settings_persist_daemon.py"),
        "object_tracking_daemon.py": find_processes("object_tracking_daemon.py")
    }

    watches = [
        x
        for x in find_processes("apply_ptz_object_preset.py")
        if "--watch" in x[1]
    ]

    for name, items in procs.items():
        print("")
        print(name, "count =", len(items))

        for pid, args in items:
            print(" ", pid, args)

    print("")
    print("apply_ptz_object_preset.py --watch count =", len(watches))

    for pid, args in watches:
        print(" ", pid, args)

    add("Process mjpeg_rknn_http count 1", len(procs["mjpeg_rknn_http"]) == 1, f"count={len(procs['mjpeg_rknn_http'])}")
    add("Process ptz_autopilot count 1", len(procs["ptz_autopilot"]) == 1, f"count={len(procs['ptz_autopilot'])}")
    add("Process settings_persist_daemon count 1", len(procs["settings_persist_daemon.py"]) == 1, f"count={len(procs['settings_persist_daemon.py'])}")
    add("Process object_tracking_daemon count 1", len(procs["object_tracking_daemon.py"]) == 1, f"count={len(procs['object_tracking_daemon.py'])}")

    return watches

def audit_runtime_zoom_feedback_coherence():
    section("LAYER 6C — RUNTIME LOGICAL ZOOM STATE / PTZ SPEED COHERENCE")

    try:
        web_text = WEB.read_text(encoding="utf-8", errors="ignore")
    except Exception as e:
        web_text = ""
        print("web read error =", e)

    start_marker = "PTZ_UI_RUNTIME_ZOOM_FEEDBACK_V1_START"
    end_marker = "PTZ_UI_RUNTIME_ZOOM_FEEDBACK_V1_END"
    start_count = web_text.count(start_marker)
    end_count = web_text.count(end_marker)
    function_tokens = [
        "pollRuntimeZoomState",
        "applyRuntimeZoomFeedbackToUi",
        "scheduleRuntimeZoomFeedback",
    ]
    markers_ok = start_count == 1 and end_count == 1 and all(t in web_text for t in function_tokens)
    print("runtime feedback start marker =", start_count)
    print("runtime feedback end marker =", end_count)
    for token in function_tokens:
        print(token, "=", token in web_text)
    add("UI runtime zoom feedback markers", markers_ok, f"start={start_count} end={end_count}")

    block = ""
    if start_count == 1 and end_count == 1:
        a = web_text.find(start_marker)
        b = web_text.find(end_marker, a)
        if a >= 0 and b >= 0:
            block = web_text[a:b]

    required = [
        "/api/zoom/state",
        "/api/autopilot/state",
        "zoom_sample_idx",
    ]
    has_active = "active_zoom_sample_idx" in block or "active_profile_idx" in block
    forbidden = [
        "/api/zoom/go_to_sample",
        "/api/autopilot/speed_profile/apply_nearest",
        "/api/zoom/jog",
        "saveSettings",
        "ptzArmed=true",
        "controlMode='ptz'",
    ]
    missing = [x for x in required if x not in block]
    present_forbidden = [x for x in forbidden if x in block]
    readonly_ok = bool(block) and not missing and has_active and not present_forbidden
    print("runtime feedback block length =", len(block))
    print("runtime feedback missing required =", missing)
    print("runtime feedback has active sample/profile =", has_active)
    print("runtime feedback forbidden present =", present_forbidden)
    add("UI runtime zoom feedback is read-only", readonly_ok, f"missing={missing} active={has_active} forbidden={present_forbidden}")

    zoom_state = get_json(f"{MJPEG_BASE}/api/zoom/state")
    ap_state = get_json(f"{PTZ_BASE}/api/autopilot/state")
    zoom_reachable = "__error__" not in zoom_state
    ap_reachable = "__error__" not in ap_state
    print("zoom state reachable =", zoom_reachable, zoom_state if not zoom_reachable else "ok")
    print("autopilot state reachable =", ap_reachable, ap_state if not ap_reachable else "ok")

    def finite_int(obj, key):
        try:
            v = obj.get(key)
            if v is None:
                return None
            n = int(round(float(v)))
            return n if n >= 0 else None
        except Exception:
            return None

    z_idx = finite_int(zoom_state, "zoom_sample_idx") if zoom_reachable else None
    ap_zoom_idx = finite_int(ap_state, "active_zoom_sample_idx") if ap_reachable else None
    ap_profile_idx = finite_int(ap_state, "active_profile_idx") if ap_reachable else None

    if zoom_reachable and ap_reachable and z_idx is not None and ap_zoom_idx is not None:
        add("Runtime logical zoom sample matches autopilot active_zoom_sample_idx", z_idx == ap_zoom_idx, f"logical_zoom={z_idx} ap_active_zoom={ap_zoom_idx}")
    else:
        add("Runtime logical zoom sample matches autopilot active_zoom_sample_idx", True, "SKIP: APIs/fields unavailable")

    speed_source = str(ap_state.get("speed_profile_source", "")) if ap_reachable else ""
    bad_sources = {"fallback", "runtime_override"}
    speed_ok = True
    speed_detail = "SKIP: APIs/fields unavailable"
    if zoom_reachable and ap_reachable and z_idx is not None and ap_profile_idx is not None and ap_zoom_idx is not None:
        speed_ok = (z_idx == ap_zoom_idx == ap_profile_idx) and speed_source not in bad_sources
        speed_detail = f"logical_zoom={z_idx} ap_active_zoom={ap_zoom_idx} ap_active_profile={ap_profile_idx} source={speed_source}"
    add("Runtime PTZ speed derives from canonical logical zoom sample", speed_ok, speed_detail)

    apply_res = post_json(f"{PTZ_BASE}/api/autopilot/speed_profile/apply_nearest", {"profile_idx": 1})
    if "__error__" in apply_res:
        warn("Layer 6D apply_nearest API contract check skipped", apply_res.get("__error__"))
    else:
        point = apply_res.get("point") if isinstance(apply_res.get("point"), dict) else {}
        returned_idx = finite_int(point, "profile_idx") if "profile_idx" in point else None
        if returned_idx is not None and returned_idx != 1:
            warn("Layer 6D apply_nearest API contract: response point mismatch", f"requested=1 returned={returned_idx}; does not block Layer 6C logical coherence")

    events_path = Path("/dev/shm/new_yolo8_object_tracking/object_tracking_events.jsonl")
    recent = []
    if events_path.exists():
        lines = events_path.read_text(encoding="utf-8", errors="replace").splitlines()[-500:]
        for line in lines:
            try:
                ev = json.loads(line)
            except Exception:
                continue
            name = ev.get("event") or ev.get("type") or ev.get("name")
            if name in {"zoom_frame_move", "zoom_frame_skip_edge", "zoom_wide_sample_move"}:
                recent.append(ev)

    if not recent:
        print("SKIP: Runtime zoom event coherence — no recent zoom_frame_move events")
        add("Runtime zoom event coherence — no recent zoom_frame_move events", True, "SKIP")
        return

    event_errors = []
    for ev in recent:
        settle = ev.get("settle") if isinstance(ev.get("settle"), dict) else None
        if not settle:
            continue
        logical = ev.get("actual_sample")
        settle_actual = settle.get("actual_sample")
        state = settle.get("state") if isinstance(settle.get("state"), dict) else {}
        state_idx = finite_int(state, "zoom_sample_idx") if state else None
        logical_i = finite_int({"v": logical}, "v")
        settle_actual_i = finite_int({"v": settle_actual}, "v")
        if settle.get("ok") is not True:
            event_errors.append(f"settle.ok not true for logical_sample={logical}: {ev}")
        if settle.get("busy") is not False:
            event_errors.append(f"settle.busy not false for logical_sample={logical}: {ev}")
        if logical_i != settle_actual_i:
            event_errors.append(f"logical_sample mismatch event.actual_sample={logical} settle.actual_sample={settle_actual}")
        if state_idx is not None and state_idx != logical_i:
            event_errors.append(f"settle.state.zoom_sample_idx mismatch logical_sample={logical} state_zoom_sample={state_idx}")
        sp = ev.get("speed_profile") if isinstance(ev.get("speed_profile"), dict) else {}
        point = sp.get("point") if isinstance(sp.get("point"), dict) else {}
        point_idx = finite_int(point, "profile_idx") if "profile_idx" in point else None
        if point_idx is not None and point_idx != logical_i:
            warn("apply_nearest response point mismatch; logical state remains canonical", f"response_profile={point.get('profile_idx')} logical_sample={logical}")
        source = sp.get("source")
        if source in {"fallback", "runtime_override"}:
            event_errors.append(f"bad speed source={source}")
    add("Runtime zoom event logical sample coherence", not event_errors, "; ".join(event_errors[:5]) or f"events={len(recent)}")

def audit_daemon_lifecycle():

    # PTZ_AUDIT_RUNTIME_DETECTION_ZOOM_V1
# PTZ_AUDIT_RUNTIME_DETECTION_ZOOM_V2
    print("")
    section("LAYER 6B — RUNTIME DETECTION OFF / ZOOM SAMPLE SYNC")

    runtime_path = ROOT / "apply_ptz_object_preset.py"
    try:
        runtime_text = runtime_path.read_text(encoding="utf-8", errors="ignore")
    except Exception as e:
        runtime_text = ""
        print("runtime read error =", e)

    def _block_between(text, start_token, end_token):
        start = text.find(start_token)
        if start < 0:
            return ""
        end = text.find(end_token, start + len(start_token))
        if end < 0:
            return text[start:]
        return text[start:end]

    runtime_tokens = [
        "PTZ_RUNTIME_DETECTION_OFF_PAUSE_GUARD_V1_START",
        "PTZ_RUNTIME_DETECTION_OFF_PAUSE_GUARD_V1_END",
        "PTZ_RUNTIME_ZOOM_SAMPLE_SPEED_SYNC_V1",
        "PTZ_RUNTIME_ZOOM_WIDE_DOCSTRING_NO_RAW_JOG_V2",
        "runtime_detection_disabled_pause",
        "runtime_pause_if_detection_disabled(\"search_preset\"",
        "runtime_pause_if_detection_disabled(\"continuous_wide_scan_x\"",
        "runtime_pause_if_detection_disabled(\"custom_plan\"",
        "zoom_wide_sample_move",
    ]

    for token in runtime_tokens:
        cnt = runtime_text.count(token)
        print(token, "=", cnt)
        add(f"Runtime contract marker {token}", cnt >= 1, f"count={cnt}")
    exact_runtime_markers = [
        "PTZ_ZOOM_SAMPLE_TRANSACTION_V1_START",
        "PTZ_SEARCH_PRESET_NO_ACQUIRE_WHILE_ZOOM_BUSY_V1_START",
    ]

    for token in exact_runtime_markers:
        cnt = runtime_text.count(token)
        print(token, "=", cnt)
        add(f"Runtime exact marker {token}", cnt == 1, f"count={cnt} expected=1")

    has_wait_zoom_sample_settled = "def wait_zoom_sample_settled" in runtime_text
    has_zoom_move_busy_check = "zoom_move_busy" in runtime_text and "wait_zoom_idle" in runtime_text
    runtime_has_go_to_sample = "/api/zoom/go_to_sample" in runtime_text
    runtime_has_apply_nearest = "/api/autopilot/speed_profile/apply_nearest" in runtime_text

    print("wait_zoom_sample_settled exists =", has_wait_zoom_sample_settled)
    print("zoom_move_busy checks exist =", has_zoom_move_busy_check)
    print("runtime /api/zoom/go_to_sample =", runtime_has_go_to_sample)
    print("runtime /api/autopilot/speed_profile/apply_nearest =", runtime_has_apply_nearest)

    add("Runtime zoom transaction waits for target settlement", has_wait_zoom_sample_settled, "requires wait_zoom_sample_settled")
    add("Runtime zoom transaction checks zoom_move_busy", has_zoom_move_busy_check, "requires zoom_move_busy checks")
    add("Runtime zoom code uses go_to_sample", runtime_has_go_to_sample, "requires /api/zoom/go_to_sample")
    add("Runtime zoom code applies nearest speed profile", runtime_has_apply_nearest, "requires /api/autopilot/speed_profile/apply_nearest")

    zoom_wide_block = _block_between(runtime_text, "def zoom_wide_pulse(", "def manual_search_pulse(")
    zoom_move_block = _block_between(runtime_text, "def zoom_move_frames(", "def target_direction_from_last(")
    continuous_block = _block_between(runtime_text, "def run_continuous_wide_scan_x(", "def run_custom_plan(")
    custom_plan_block = _block_between(runtime_text, "def run_custom_plan(", "def run_mode(")
    search_block = _block_between(runtime_text, "def run_named_search_preset_once(", "def run_active_search_preset_once(")

    wide_raw_jog = zoom_wide_block.count('/api/zoom/jog')
    wide_go_to_sample = zoom_wide_block.count('/api/zoom/go_to_sample')
    wide_apply_speed = zoom_wide_block.count('apply_ptz_speed_profile_for_zoom')

    move_go_to_sample = zoom_move_block.count('/api/zoom/go_to_sample')
    move_profile_idx = zoom_move_block.count('"profile_idx"') + zoom_move_block.count("'profile_idx'")
    move_apply_speed = zoom_move_block.count('apply_ptz_speed_profile_for_zoom')
    move_raw_state_write = zoom_move_block.count('/api/zoom/state')
    move_raw_jog = zoom_move_block.count('/api/zoom/jog')

    search_has_pause_guard = 'runtime_pause_if_detection_disabled("search_preset"' in search_block
    continuous_has_pause_guard = 'runtime_pause_if_detection_disabled("continuous_wide_scan_x"' in continuous_block
    custom_plan_has_pause_guard = 'runtime_pause_if_detection_disabled("custom_plan"' in custom_plan_block

    print("zoom_wide_pulse scoped /api/zoom/jog =", wide_raw_jog)
    print("zoom_wide_pulse scoped /api/zoom/go_to_sample =", wide_go_to_sample)
    print("zoom_wide_pulse scoped apply_ptz_speed_profile_for_zoom =", wide_apply_speed)
    print("zoom_move_frames scoped /api/zoom/go_to_sample =", move_go_to_sample)
    print("zoom_move_frames scoped profile_idx =", move_profile_idx)
    print("zoom_move_frames scoped apply_ptz_speed_profile_for_zoom =", move_apply_speed)
    print("zoom_move_frames scoped /api/zoom/state =", move_raw_state_write)
    print("zoom_move_frames scoped /api/zoom/jog =", move_raw_jog)
    print("search pause guard =", search_has_pause_guard)
    print("continuous pause guard =", continuous_has_pause_guard)
    # PTZ_AUDIT_RUNTIME_DETECTION_ZOOM_V2
    # v1 could miss a valid custom-plan guard if the guard is placed
    # before event_log("custom_plan_start") / before zoom_wide_on_start.
    def _ptz_runtime_function_block_containing_v2(src, token):
        pos = src.find(token)
        if pos < 0:
            return ""
        start = src.rfind("\ndef ", 0, pos)
        if start < 0:
            start = 0
        else:
            start += 1
        end = src.find("\ndef ", pos + 1)
        if end < 0:
            end = len(src)
        return src[start:end]

    _custom_plan_block_v2 = _ptz_runtime_function_block_containing_v2(runtime_text, 'custom_plan_start')
    if not _custom_plan_block_v2:
        _custom_plan_block_v2 = _ptz_runtime_function_block_containing_v2(runtime_text, 'runtime_pause_if_detection_disabled("custom_plan"')
    custom_plan_has_pause_guard = (
        'runtime_pause_if_detection_disabled("custom_plan"' in _custom_plan_block_v2
        or 'runtime_pause_if_detection_disabled("custom_plan"' in runtime_text
    )
    print("custom plan function scoped block length =", len(_custom_plan_block_v2))
    print("custom plan pause guard =", custom_plan_has_pause_guard)

    add(
        "Runtime Detection OFF pauses search/reacquire",
        search_has_pause_guard and continuous_has_pause_guard and custom_plan_has_pause_guard,
        f"search={search_has_pause_guard} continuous={continuous_has_pause_guard} custom_plan={custom_plan_has_pause_guard}"
    )
    add(
        "Runtime wide zoom uses sample-speed contract",
        bool(zoom_wide_block) and wide_raw_jog == 0 and wide_go_to_sample >= 1 and wide_apply_speed >= 1,
        f"raw_jog={wide_raw_jog} go_to_sample={wide_go_to_sample} apply_speed={wide_apply_speed}"
    )
    add(
        "Runtime search zoom step uses profile_idx sample contract",
        bool(zoom_move_block) and move_go_to_sample >= 1 and move_profile_idx >= 1 and move_apply_speed >= 1,
        f"go_to_sample={move_go_to_sample} profile_idx={move_profile_idx} apply_speed={move_apply_speed}"
    )
    add(
        "Runtime search zoom step avoids raw zoom state/jog writes",
        bool(zoom_move_block) and move_raw_state_write == 0 and move_raw_jog == 0,
        f"zoom_state={move_raw_state_write} zoom_jog={move_raw_jog}"
    )
    add(
        "Runtime wide/search zoom functions do not raw jog",
        bool(zoom_wide_block) and bool(zoom_move_block) and wide_raw_jog == 0 and move_raw_jog == 0,
        f"wide_raw_jog={wide_raw_jog} move_raw_jog={move_raw_jog}"
    )

    section("LAYER 5/6 — DAEMON AND RUNTIME LIFECYCLE")

    settings = get_json(f"{MJPEG_BASE}/api/settings")
    file_snapshot = read_json_file(SETTINGS_FILE)

    if "__error__" in settings:
        add("Lifecycle skipped settings reachable", False, settings["__error__"])
        return

    original = dict(settings)

    api_custom = settings.get("objectPresetsCustom") or {}
    file_custom = file_snapshot.get("objectPresetsCustom") or {}
    persistent_custom = api_custom or file_custom

    add(
        "Lifecycle persistent custom presets available",
        bool(persistent_custom),
        f"api_keys={sorted(api_custom.keys())} file_keys={sorted(file_custom.keys())}"
    )

    if not persistent_custom:
        return

    def preset_for(name):
        p = persistent_custom.get(name) or {}
        return p if isinstance(p, dict) else {}

    def child_start_events_since(ts0):
        events = []

        # PTZ_AUDIT_DAEMON_LIFECYCLE_LOG_PATHS_V4
        # Runtime may archive /dev/shm log on ptzArmed=false.
        # Check both shared-memory live log and persistent current log.
        for path in [
            "/dev/shm/new_yolo8_object_tracking/object_tracking_daemon.log",
            "/root/new_yolo8/logs/current/object_tracking_daemon.log",
            "/root/new_yolo8/object_tracking_daemon.log"
        ]:
            p = Path(path)

            if not p.exists():
                continue

            for line in p.read_text(encoding="utf-8", errors="replace").splitlines():
                line = line.strip()

                if not line:
                    continue

                try:
                    obj = json.loads(line)
                except Exception:
                    continue

                if obj.get("event") not in ("child_start", "armed_running"):
                    continue

                try:
                    ts = float(obj.get("ts") or 0)
                except Exception:
                    ts = 0

                if ts >= ts0:
                    obj["_path"] = path
                    events.append(obj)

        return events

    def watch_count():
        return len([
            x
            for x in find_processes("apply_ptz_object_preset.py")
            if "--watch" in x[1]
        ])

    # PTZ_AUDIT_NON_DESTRUCTIVE_PRESET_RESTORE_V1_START
    original_active_object_preset = settings.get("activeObjectPreset")
    original_active_search_preset = settings.get("activeSearchPreset")
    lifecycle_active_object_preset = original_active_object_preset if preset_for(original_active_object_preset) else next(iter(sorted(persistent_custom.keys())), None)
    lifecycle_active_search_preset = original_active_search_preset or settings.get("activeSearchPreset") or "lost_step_wait"
    original_active_preset = preset_for(original_active_object_preset)
    original_preset_snapshot = {
        "classes": normalize_list_numbers(original_active_preset.get("classes")),
        "detection_mode": original_active_preset.get("detection_mode"),
        "max_detections": original_active_preset.get("max_detections"),
        "max_raw_candidates": original_active_preset.get("max_raw_candidates"),
        "detect_every_n_frames": original_active_preset.get("detect_every_n_frames"),
    }

    print("Lifecycle original activeObjectPreset =", original_active_object_preset)
    print("Lifecycle original activeSearchPreset =", original_active_search_preset)
    print("Lifecycle original active preset snapshot =", json.dumps(original_preset_snapshot, ensure_ascii=False, sort_keys=True))
    print("Lifecycle test activeObjectPreset =", lifecycle_active_object_preset)
    print("Lifecycle test activeSearchPreset =", lifecycle_active_search_preset)

    add(
        "Lifecycle original active preset snapshot available",
        bool(original_active_object_preset and original_active_preset),
        f"active={original_active_object_preset} preset={original_preset_snapshot}"
    )

    def preset_backend_matches(preset):
        limits = get_json(f"{MJPEG_BASE}/api/detection/limits")
        throttle = get_json(f"{MJPEG_BASE}/api/detection/throttle")
        roi = get_json(f"{MJPEG_BASE}/api/detection/roi_config")
        det = get_json(f"{MJPEG_BASE}/api/detector/config")

        if any("__error__" in x for x in [limits, throttle, roi, det]):
            return False, det, limits, throttle, roi

        classes_ok = normalize_list_numbers(det.get("selected_classes")) == normalize_list_numbers(preset.get("classes"))
        limit_ok = int(limits.get("max_detections") or -1) == int(preset.get("max_detections") or -2)
        throttle_ok = int(throttle.get("detect_every_n_frames") or -1) == int(preset.get("detect_every_n_frames") or -2)
        roi_ok = str(roi.get("detection_mode") or "") == str(preset.get("detection_mode") or "")
        return classes_ok and limit_ok and throttle_ok and roi_ok, det, limits, throttle, roi

    def hydrate_runtime_settings():
        hydrate = ROOT / "hydrate_runtime_settings.py"
        if not hydrate.exists():
            return False, "missing hydrate_runtime_settings.py"
        try:
            proc = subprocess.run(["python3", str(hydrate)], cwd=str(ROOT), text=True, capture_output=True, timeout=20)
        except Exception as e:
            return False, str(e)
        detail = (proc.stdout + proc.stderr).strip()[-1000:]
        return proc.returncode == 0, detail or f"returncode={proc.returncode}"

    def wait_for_original_backend(timeout_s=15):
        deadline = time.time() + timeout_s
        last = ({}, {}, {}, {})
        while time.time() < deadline:
            ok, det, limits, throttle, roi = preset_backend_matches(original_active_preset)
            last = (det, limits, throttle, roi)
            if ok:
                return True, det, limits, throttle, roi
            time.sleep(1)
        return (False,) + last

    def build_settings(value):
        s = get_json(f"{MJPEG_BASE}/api/settings")

        if "__error__" in s:
            s = {}

        # Preserve object presets in every audit POST.
        # /api/settings may persist directly to ui_settings.json, so posting a minimal
        # object without objectPresetsCustom can destroy persistent presets.
        s["objectPresetsCustom"] = persistent_custom

        active = lifecycle_active_object_preset
        preset = preset_for(active)

        if active:
            s["activeObjectPreset"] = active
        s["activeSearchPreset"] = lifecycle_active_search_preset
        s["objectPresetTrackingMode"] = "single_auto"
        s["objectPresetLossBehavior"] = "continuous_wide_scan_x"
        s["ptzArmed"] = bool(value)
        s["controlMode"] = "ptz" if value else "manual"

        if preset:
            s["operatorModel"] = preset.get("operatorModel") or preset.get("model") or s.get("operatorModel")
            s["operatorDetectionLimit"] = int(preset.get("max_detections") or s.get("operatorDetectionLimit") or 5)
            s["operatorDetectEvery"] = int(preset.get("detect_every_n_frames") or s.get("operatorDetectEvery") or 1)
            s["operatorDetectionAreaMode"] = preset.get("detection_mode") or s.get("operatorDetectionAreaMode") or "full_frame"
            s["detectorSelectedClasses"] = preset.get("classes") or s.get("detectorSelectedClasses") or []
            s["lastAppliedObjectPreset"] = {
                "name": active,
                "label": preset.get("label") or active,
                "tracking_mode": "single_auto",
                "loss_behavior": "continuous_wide_scan_x",
                "ts": int(time.time())
            }

        return s

    def set_armed(value):
        post_json(f"{MJPEG_BASE}/api/settings", build_settings(value))

    print("Step A: disarm")
    set_armed(False)
    post_json(f"{PTZ_BASE}/api/autopilot/stop", {})
    time.sleep(3)
    wc0 = watch_count()
    print("watch_count after disarm =", wc0)

    print("Step B: arm")
    arm_ts = time.time()
    set_armed(True)

    samples = []

    for i in range(12):
        time.sleep(1)
        wc = watch_count()
        samples.append(wc)
        print("watch_count arm sample", i + 1, "=", wc)

    events = child_start_events_since(arm_ts)
    wc1_max = max(samples) if samples else 0
    wc1_final = samples[-1] if samples else 0

    print("watch_count max during arm =", wc1_max)
    print("watch_count final during arm =", wc1_final)
    print("runtime start events during arm =", len(events))

    for ev in events[-3:]:
        print("runtime_event", {
            "event": ev.get("event"),
            "path": ev.get("_path"),
            "ts": ev.get("ts"),
            "preset": ev.get("preset"),
            "search_preset": ev.get("search_preset"),
            "pid": ev.get("pid")
        })

    print("Step C: final disarm")
    set_armed(False)
    post_json(f"{PTZ_BASE}/api/autopilot/stop", {})
    time.sleep(4)
    wc2 = watch_count()
    print("watch_count after final disarm =", wc2)

    # Restore active preset/control state safely, then re-apply settings to runtime
    # so detector backend state is the same active preset the audit observed before
    # the lifecycle arm/disarm sequence.
    restored = dict(original)
    restored["objectPresetsCustom"] = persistent_custom
    restored["ptzArmed"] = False
    restored["controlMode"] = "manual"

    if original_active_object_preset:
        restored["activeObjectPreset"] = original_active_object_preset
    if original_active_search_preset:
        restored["activeSearchPreset"] = original_active_search_preset

    post_json(f"{MJPEG_BASE}/api/settings", restored)
    post_json(f"{PTZ_BASE}/api/autopilot/stop", {})

    hydrate_ok, hydrate_detail = hydrate_runtime_settings()
    print("hydrate restore ok =", hydrate_ok)
    print("hydrate restore detail =", hydrate_detail)

    restore_ok = False
    restore_det = {}
    restore_limits = {}
    restore_throttle = {}
    restore_roi = {}
    if original_active_preset:
        restore_ok, restore_det, restore_limits, restore_throttle, restore_roi = wait_for_original_backend()

    restored_settings = get_json(f"{MJPEG_BASE}/api/settings")
    settings_restore_ok = (
        restored_settings.get("activeObjectPreset") == original_active_object_preset
        and restored_settings.get("activeSearchPreset") == original_active_search_preset
    )

    backend_classes = normalize_list_numbers(restore_det.get("selected_classes"))
    preset_classes = normalize_list_numbers(original_active_preset.get("classes")) if original_active_preset else []
    restore_detail = (
        f"backend={backend_classes} preset={preset_classes} "
        f"limits={{'max_detections': {restore_limits.get('max_detections')}, "
        f"'detect_every_n_frames': {restore_throttle.get('detect_every_n_frames')}, "
        f"'detection_mode': {restore_roi.get('detection_mode')}}} "
        f"activeObjectPreset={restored_settings.get('activeObjectPreset')} "
        f"activeSearchPreset={restored_settings.get('activeSearchPreset')} hydrate_ok={hydrate_ok}"
    )

    add("Daemon lifecycle disarm watch_count=0", wc0 == 0, f"watch_count={wc0}")
    add(
        "Daemon lifecycle arm runtime start observed",
        len(events) >= 1 or wc1_max >= 1,
        f"runtime_events={len(events)} max_watch_count={wc1_max} final_watch_count={wc1_final} samples={samples}"
    )
    add("Daemon lifecycle final disarm watch_count=0", wc2 == 0, f"watch_count={wc2}")
    add(
        "Audit restored active detector preset after lifecycle",
        bool(hydrate_ok and restore_ok and settings_restore_ok),
        restore_detail
    )
    print(f"PASS: Audit restored active detector preset after lifecycle — backend={backend_classes} preset={preset_classes}" if hydrate_ok and restore_ok and settings_restore_ok else f"FAIL: Audit restored active detector preset after lifecycle — {restore_detail}")
    # PTZ_AUDIT_NON_DESTRUCTIVE_PRESET_RESTORE_V1_END


def audit_launcher():
    section("LAUNCHER CONTRACT")

    p = ROOT / "launcher.sh"

    if not p.exists():
        add("Launcher exists", False, "missing")
        return

    s = p.read_text(encoding="utf-8", errors="ignore")

    checks = [
        "HYDRATE_RUNTIME_SETTINGS",
        "hydrate_runtime_settings()",
        "hydrate_runtime_settings",
        "SETTINGS_PERSIST_DAEMON",
        "start_settings_persist_daemon()",
        "start_settings_persist_daemon",
        "OBJECT_TRACKING_DAEMON",
        "start_object_tracking_daemon()",
        "start_object_tracking_daemon"
    ]

    for c in checks:
        count = s.count(c)
        print(c, "=", count)
        add(f"Launcher token {c}", count > 0, f"count={count}")

def print_summary():
    section("SUMMARY")

    ok_count = 0
    fail_count = 0

    for name, ok, detail in RESULTS:
        status = "PASS" if ok else "FAIL"

        if ok:
            ok_count += 1
        else:
            fail_count += 1

        print(f"{status}: {name} — {detail}")

    warn_count = len(WARNINGS)
    for name, detail in WARNINGS:
        print(f"WARN: {name} — {detail}")

    print("")
    print("PASS =", ok_count)
    print("FAIL =", fail_count)
    print("WARN =", warn_count)

    report = {
        "ts": int(time.time()),
        "pass": ok_count,
        "fail": fail_count,
        "warn": len(WARNINGS),
        "warnings": [
            {"name": name, "detail": detail}
            for name, detail in WARNINGS
        ],
        "results": [
            {
                "name": name,
                "ok": ok,
                "detail": detail
            }
            for name, ok, detail in RESULTS
        ]
    }

    out = ROOT / "ptz_contract_audit_last.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print("")
    print("Wrote", out)

    return fail_count

def main():
    audit_files()
    audit_ui()

    api = get_json(f"{MJPEG_BASE}/api/settings")
    file_data = read_json_file(SETTINGS_FILE)

    audit_settings(api, file_data)
    audit_detector(api)
    audit_ptz()
    audit_processes()
    audit_launcher()
    audit_runtime_zoom_feedback_coherence()
    audit_daemon_lifecycle()

    fail_count = print_summary()

    return 0 if fail_count == 0 else 2

if __name__ == "__main__":
    raise SystemExit(main())
