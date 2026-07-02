#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
AUDIT = ROOT / 'ptz_contract_audit.py'
MARKER = 'PTZ_AUDIT_RUNTIME_DETECTION_ZOOM_V1'

block = r'''

    # PTZ_AUDIT_RUNTIME_DETECTION_ZOOM_V1
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
'''

if not AUDIT.exists():
    raise SystemExit(f'Missing {AUDIT}')

text = AUDIT.read_text(encoding='utf-8', errors='ignore')
if MARKER in text:
    print('Already patched:', MARKER)
    raise SystemExit(0)

backup = AUDIT.with_name(AUDIT.name + f'.bak_runtime_detection_zoom_v1_{int(time.time())}')
backup.write_text(text, encoding='utf-8')

needle = 'section("LAYER 5/6 — DAEMON AND RUNTIME LIFECYCLE")'
pos = text.find(needle)
if pos < 0:
    needle = "section('LAYER 5/6 — DAEMON AND RUNTIME LIFECYCLE')"
    pos = text.find(needle)
if pos < 0:
    raise SystemExit('Could not find LAYER 5/6 insertion point in ptz_contract_audit.py')

# Insert immediately before the LAYER 5/6 section so runtime contract is checked before lifecycle arming mutates state.
line_start = text.rfind('\n', 0, pos)
if line_start < 0:
    line_start = pos
new_text = text[:line_start] + block + text[line_start:]
AUDIT.write_text(new_text, encoding='utf-8')

print('OK patched ptz_contract_audit.py runtime detection-off / zoom sync audit v1')
print('Backup:', backup)
print('Changes:')
print(' - added LAYER 6B — RUNTIME DETECTION OFF / ZOOM SAMPLE SYNC')
print(' - checks Detection OFF pause guards before search/reacquire')
print(' - checks wide zoom uses go_to_sample + apply_nearest and no raw jog')
print(' - checks search zoom steps use profile_idx sample contract')