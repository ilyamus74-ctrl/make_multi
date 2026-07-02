#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
AUDIT = ROOT / 'ptz_contract_audit.py'
MARKER = 'PTZ_AUDIT_KEYBOARD_QA_SWEEP_V1'

block = r'''

    # PTZ_AUDIT_KEYBOARD_QA_SWEEP_V1
    print("")
    section("LAYER 1C — KEYBOARD Q/A SWEEP JS CONTRACT")

    try:
        _qa_web_text = web_text
    except Exception:
        _qa_web_text = (ROOT / "web/index.html").read_text(encoding="utf-8", errors="ignore")

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
    has_no_canonical_sample_idx = "sample_idx: target" not in _qa_web_text
    has_apply_nearest = "/api/autopilot/speed_profile/apply_nearest" in _qa_web_text or "speed_profile/apply_nearest" in _qa_web_text
    has_profile_idx_final = "profile_idx: finalIdx" in _qa_web_text
    has_capture_stop = "stopImmediatePropagation" in _qa_web_text and "ptzQaSweepIsSweepMode" in _qa_web_text

    print("Q/A go_to_sample backend call =", has_go_to_sample)
    print("Q/A go_to_sample uses profile_idx target =", has_profile_idx_target)
    print("Q/A avoids canonical sample_idx target =", has_no_canonical_sample_idx)
    print("Q/A apply_nearest backend call =", has_apply_nearest)
    print("Q/A apply_nearest uses profile_idx finalIdx =", has_profile_idx_final)
    print("Q/A capture stop =", has_capture_stop)

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
'''

if not AUDIT.exists():
    raise SystemExit(f'Missing {AUDIT}')

text = AUDIT.read_text(encoding='utf-8', errors='ignore')
if MARKER in text:
    print('Already patched:', MARKER)
    raise SystemExit(0)

backup = AUDIT.with_name(AUDIT.name + f'.bak_keyboard_qa_sweep_v1_{int(time.time())}')
backup.write_text(text, encoding='utf-8')

layer_pos = text.find('LAYER 1 — BROWSER UI')
if layer_pos < 0:
    raise SystemExit('Could not find LAYER 1 — BROWSER UI in ptz_contract_audit.py')

def_pos = text.rfind('\ndef ', 0, layer_pos)
if def_pos < 0:
    def_pos = text.rfind('def ', 0, layer_pos)
if def_pos < 0:
    raise SystemExit('Could not find containing function for LAYER 1')

next_def = text.find('\ndef ', layer_pos)
if next_def < 0:
    raise SystemExit('Could not find insertion boundary after LAYER 1 function')

new_text = text[:next_def] + block + text[next_def:]
AUDIT.write_text(new_text, encoding='utf-8')

print('OK patched ptz_contract_audit.py keyboard Q/A sweep audit v1')
print('Backup:', backup)
print('Changes:')
print(' - added LAYER 1C — KEYBOARD Q/A SWEEP JS CONTRACT')
print(' - checks Q/A profile_idx backend zoom movement')
print(' - checks PTZ speed profile apply_nearest by profile_idx')