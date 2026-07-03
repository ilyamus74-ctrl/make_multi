#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
WEB = ROOT / 'web' / 'index.html'
MARKER_START = 'PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_START'
MARKER_END = 'PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_END'
MARKER_V2 = 'PTZ_QA_SAMPLE_STEP_MOVEMENT_MODE_V2'

if not WEB.exists():
    raise SystemExit(f'Missing {WEB}')

text = WEB.read_text(encoding='utf-8', errors='ignore')
if MARKER_V2 in text:
    print('Already patched:', MARKER_V2)
    raise SystemExit(0)
if MARKER_START not in text or MARKER_END not in text:
    raise SystemExit('Missing Q/A profile_idx v1 block; apply patch_ui_qa_sweep_profile_idx_v1.py first')

backup = WEB.with_name(WEB.name + f'.bak_qa_sample_step_movement_mode_v2_{int(time.time())}')
backup.write_text(text, encoding='utf-8')

start = text.find(MARKER_START)
end = text.find(MARKER_END, start)
if start < 0 or end < 0 or end <= start:
    raise SystemExit('Could not locate canonical Q/A block')
block = text[start:end]

old = """  function ptzQaSweepIsSweepMode() {\n    return ($('zoomMoveMode')?.value || '') === 'sweep_time_steps';\n  }\n"""
new = """  // PTZ_QA_SAMPLE_STEP_MOVEMENT_MODE_V2\n  // Q/A is a sample-step command. ZOOM CALIB Movement selects the physical move mode:\n  // legacy_impulse or sweep_time_steps. Browser chooses only target profile_idx.\n  function ptzQaSweepMovementMode() {\n    const raw = String($('zoomMoveMode')?.value || 'legacy_impulse');\n    return raw === 'sweep_time_steps' ? 'sweep_time_steps' : 'legacy_impulse';\n  }\n\n  function ptzQaSweepIsSweepMode() {\n    return ptzQaSweepMovementMode() === 'sweep_time_steps';\n  }\n"""
if old not in block:
    raise SystemExit('Could not find ptzQaSweepIsSweepMode function in canonical block')
block = block.replace(old, new, 1)

old = """      const moveRes = await apiPostJson('/api/zoom/go_to_sample', {\n        profile_idx: target,\n        mode: 'relative',\n        source\n      });\n"""
new = """      const movementMode = ptzQaSweepMovementMode();\n\n      const moveRes = await apiPostJson('/api/zoom/go_to_sample', {\n        profile_idx: target,\n        mode: 'relative',\n        movement_mode: movementMode,\n        zoom_move_mode: movementMode,\n        source\n      });\n"""
if old not in block:
    raise SystemExit('Could not find go_to_sample payload in canonical block')
block = block.replace(old, new, 1)

old = """      if ($('ptzTuneStatus')) {\n        $('ptzTuneStatus').textContent = `Q/A profile_idx sweep ${st.idx}→${finalIdx} source=${source}`;\n      }\n"""
new = """      if ($('ptzTuneStatus')) {\n        $('ptzTuneStatus').textContent = `Q/A profile_idx step ${st.idx}→${finalIdx} movement=${movementMode} source=${source}`;\n      }\n"""
if old in block:
    block = block.replace(old, new, 1)

old = """          moveRes,\n          speedRes\n"""
new = """          movementMode,\n          moveRes,\n          speedRes\n"""
if old in block:
    block = block.replace(old, new, 1)

# Canonical Q/A must own both ZOOM CALIB movement modes. Backend go_to_sample decides physical movement.
count = block.count("    if (!ptzQaSweepIsSweepMode()) return;\n")
if count < 2:
    raise SystemExit(f'Expected two sweep-only guards in canonical block, found {count}')
block = block.replace(
    "    if (!ptzQaSweepIsSweepMode()) return;\n",
    "    // V2: own Q/A in both ZOOM CALIB Movement modes; backend go_to_sample decides physical movement.\n",
)

new_text = text[:start] + block + text[end:]
WEB.write_text(new_text, encoding='utf-8')

print('OK patched web/index.html Q/A sample-step movement mode v2')
print('Backup:', backup)
print('Changes:')
print(' - Q/A canonical handler now owns both ZOOM CALIB Movement modes')
print(' - Q/A chooses profile_idx target only')
print(' - /api/zoom/go_to_sample receives movement_mode/zoom_move_mode for diagnostics')
print(' - backend remains source of physical movement: legacy_impulse or sweep_time_steps')