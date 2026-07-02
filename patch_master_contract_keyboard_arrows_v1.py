#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
MD = ROOT / 'PTZ_MASTER_CONTRACT.md'
MARKER = 'Layer 1D Additional Contract — Keyboard Arrow PTZ Pan/Tilt'

section = r'''

## Layer 1D Additional Contract — Keyboard Arrow PTZ Pan/Tilt

Arrow keys are operator pan/tilt controls and must be tested as part of the browser keyboard contract.

Required behavior:

- ArrowLeft, ArrowRight, ArrowUp, ArrowDown are owned by the UI only in PTZ mode.
- Arrow keydown must capture the event before browser/page handlers, call preventDefault, stopPropagation, and stopImmediatePropagation.
- Arrow keydown must start or continue real backend/manual pan/tilt movement, not only update visible UI state.
- Arrow keyup must release the corresponding key state and stop or debounce-stop backend/manual movement.
- Arrow controls must not interfere with Q/A zoom sample stepping.
- Arrow controls must not fire while normal text/input editing owns focus unless the dedicated PTZ capture logic intentionally blurs/owns the control in PTZ mode.

Audit baseline:

- LAYER 1D — KEYBOARD ARROW PTZ JS CONTRACT must pass.
'''

if not MD.exists():
    raise SystemExit(f'Missing {MD}')

text = MD.read_text(encoding='utf-8', errors='ignore')
if MARKER in text:
    print('Already patched:', MARKER)
    raise SystemExit(0)

backup = MD.with_name(MD.name + f'.bak_keyboard_arrows_v1_{int(time.time())}')
backup.write_text(text, encoding='utf-8')

# Put the arrow contract after the Q/A keyboard section when available.
pos = text.find('Layer 1C Clarification — Q/A Sample Step Uses ZOOM CALIB Movement')
if pos >= 0:
    next_h2 = text.find('\n## ', pos + 1)
    if next_h2 >= 0:
        text = text[:next_h2] + section + text[next_h2:]
    else:
        text = text.rstrip() + section + '\n'
else:
    text = text.rstrip() + section + '\n'

MD.write_text(text, encoding='utf-8')
print('OK patched PTZ_MASTER_CONTRACT.md keyboard arrows contract v1')
print('Backup:', backup)