#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
AUDIT = ROOT / 'ptz_contract_audit.py'
MARKER_V1 = 'PTZ_AUDIT_KEYBOARD_QA_SWEEP_V1'
MARKER_V2 = 'PTZ_AUDIT_KEYBOARD_QA_SWEEP_V2'

if not AUDIT.exists():
    raise SystemExit(f'Missing {AUDIT}')

text = AUDIT.read_text(encoding='utf-8', errors='ignore')
if MARKER_V2 in text:
    print('Already patched:', MARKER_V2)
    raise SystemExit(0)
if MARKER_V1 not in text:
    raise SystemExit(f'Missing {MARKER_V1}; apply v1 audit patch first')

backup = AUDIT.with_name(AUDIT.name + f'.bak_keyboard_qa_sweep_v2_{int(time.time())}')
backup.write_text(text, encoding='utf-8')

# Add V2 marker near V1 marker.
text = text.replace(
    f'# {MARKER_V1}',
    f'# {MARKER_V1}\n    # {MARKER_V2}',
    1,
)

needle = '    qa_tokens = [\n'
pos = text.find(needle, text.find(MARKER_V2))
if pos < 0:
    raise SystemExit('Could not find qa_tokens block')

insert = '''    qa_start = _qa_web_text.find("PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_START")\n    qa_end = _qa_web_text.find("PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_END", qa_start)\n    if qa_start >= 0 and qa_end > qa_start:\n        qa_block = _qa_web_text[qa_start:qa_end]\n    else:\n        qa_block = ""\n\n'''
text = text[:pos] + insert + text[pos:]

old = '    has_no_canonical_sample_idx = "sample_idx: target" not in _qa_web_text\n'
new = '''    # Scope this check to the canonical Q/A block only.\n    # Other legacy/sample UI code can legitimately contain sample_idx,\n    # but Q/A must use profile_idx as the backend request key.\n    has_no_canonical_sample_idx = (\n        bool(qa_block)\n        and "sample_idx:" not in qa_block\n        and "\\\"sample_idx\\\"" not in qa_block\n        and "'sample_idx'" not in qa_block\n    )\n'''
if old not in text:
    raise SystemExit('Could not find has_no_canonical_sample_idx line')
text = text.replace(old, new, 1)

old = '    has_capture_stop = "stopImmediatePropagation" in _qa_web_text and "ptzQaSweepIsSweepMode" in _qa_web_text\n'
new = '''    has_capture_stop = (\n        bool(qa_block)\n        and "stopImmediatePropagation" in qa_block\n        and "window.addEventListener('keydown'" in qa_block\n        and "}, true)" in qa_block\n        and ("sweep_time_steps" in qa_block or "ptzQaSweepIsSweepMode" in qa_block or "ptzQaSweepIsSweep" in qa_block)\n    )\n'''
if old not in text:
    raise SystemExit('Could not find has_capture_stop line')
text = text.replace(old, new, 1)

# Make diagnostics explicit.
text = text.replace(
    '    print("Q/A go_to_sample backend call =", has_go_to_sample)\n',
    '    print("Q/A scoped block length =", len(qa_block))\n    print("Q/A go_to_sample backend call =", has_go_to_sample)\n',
    1,
)

AUDIT.write_text(text, encoding='utf-8')

print('OK patched ptz_contract_audit.py keyboard Q/A sweep audit v2')
print('Backup:', backup)
print('Changes:')
print(' - scopes sample_idx prohibition to PTZ_QA_SWEEP_PROFILE_IDX_BACKEND block')
print(' - validates capture stop inside the canonical Q/A block')
print(' - avoids false FAIL from old legacy sample code')