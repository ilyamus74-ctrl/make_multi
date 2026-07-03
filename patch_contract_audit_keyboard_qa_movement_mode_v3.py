#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
AUDIT = ROOT / 'ptz_contract_audit.py'
MARKER_V2 = 'PTZ_AUDIT_KEYBOARD_QA_SWEEP_V2'
MARKER_V3 = 'PTZ_AUDIT_KEYBOARD_QA_MOVEMENT_MODE_V3'

if not AUDIT.exists():
    raise SystemExit(f'Missing {AUDIT}')

text = AUDIT.read_text(encoding='utf-8', errors='ignore')
if MARKER_V3 in text:
    print('Already patched:', MARKER_V3)
    raise SystemExit(0)
if MARKER_V2 not in text:
    raise SystemExit(f'Missing {MARKER_V2}; apply keyboard Q/A audit v2 first')

backup = AUDIT.with_name(AUDIT.name + f'.bak_keyboard_qa_movement_mode_v3_{int(time.time())}')
backup.write_text(text, encoding='utf-8')

text = text.replace(
    f'# {MARKER_V2}',
    f'# {MARKER_V2}\n    # {MARKER_V3}',
    1,
)

needle = '    print("Q/A scoped block length =", len(qa_block))\n'
insert = '''    has_movement_mode_v2 = "PTZ_QA_SAMPLE_STEP_MOVEMENT_MODE_V2" in qa_block
    qa_owns_both_zoom_movement_modes = (
        bool(qa_block)
        and "if (!ptzQaSweepIsSweepMode()) return" not in qa_block
        and "legacy_impulse" in qa_block
        and "sweep_time_steps" in qa_block
        and "ptzQaSweepMovementMode" in qa_block
    )

'''
if needle not in text:
    raise SystemExit('Could not find Q/A scoped block print')
text = text.replace(needle, insert + needle, 1)

needle = '    print("Q/A capture stop =", has_capture_stop)\n'
insert = '''    print("Q/A movement mode v2 marker =", has_movement_mode_v2)
    print("Q/A owns both ZOOM CALIB movement modes =", qa_owns_both_zoom_movement_modes)
'''
if needle not in text:
    raise SystemExit('Could not find Q/A capture stop print')
text = text.replace(needle, needle + insert, 1)

needle = '''    add(
        "Keyboard Q/A sweep owns event before older handlers",
        has_capture_stop,
        f"capture_stop={has_capture_stop}"
    )
'''
insert = '''    add(
        "Keyboard Q/A sample step respects ZOOM CALIB Movement",
        has_movement_mode_v2 and qa_owns_both_zoom_movement_modes,
        f"movement_marker={has_movement_mode_v2} owns_both_modes={qa_owns_both_zoom_movement_modes}"
    )
'''
if needle not in text:
    raise SystemExit('Could not find final Q/A capture add block')
text = text.replace(needle, needle + insert, 1)

AUDIT.write_text(text, encoding='utf-8')

print('OK patched ptz_contract_audit.py keyboard Q/A movement mode audit v3')
print('Backup:', backup)
print('Changes:')
print(' - verifies canonical Q/A block owns both ZOOM CALIB Movement modes')
print(' - verifies handler no longer gates Q/A only to sweep_time_steps')
print(' - keeps backend profile_idx contract checks from v2')