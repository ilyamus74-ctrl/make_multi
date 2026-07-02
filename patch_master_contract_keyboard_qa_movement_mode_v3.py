#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
MD = ROOT / 'PTZ_MASTER_CONTRACT.md'
TITLE = 'Layer 1C Clarification — Q/A Sample Step Uses ZOOM CALIB Movement'

if not MD.exists():
    raise SystemExit(f'Missing {MD}')

text = MD.read_text(encoding='utf-8', errors='ignore')
if TITLE in text:
    print('Already present:', TITLE)
    raise SystemExit(0)

backup = MD.with_name(MD.name + f'.bak_keyboard_qa_movement_mode_v3_{int(time.time())}')
backup.write_text(text, encoding='utf-8')

append = f'''

## {TITLE}

Q/A keyboard control is a zoom sample-step command, not a separate movement-mode selector.

Required behavior:

- `Q` selects target `profile_idx + 1`.
- `A` selects target `profile_idx - 1`.
- Browser sends `POST /api/zoom/go_to_sample` with `profile_idx` and `mode=relative`.
- The physical movement method is selected by `ZOOM CALIB → Movement`:
  - `LEGACY IMPULSE`
  - `SWEEP TIME STEPS`
- Browser must not restrict canonical Q/A handling only to `SWEEP TIME STEPS`.
- Browser may include `movement_mode`/`zoom_move_mode` for diagnostics, but backend remains the source of physical movement.
- After movement, browser applies `/api/autopilot/speed_profile/apply_nearest` by `profile_idx`.

Audit coverage:

- marker `PTZ_QA_SAMPLE_STEP_MOVEMENT_MODE_V2`
- marker `PTZ_AUDIT_KEYBOARD_QA_MOVEMENT_MODE_V3`
'''

MD.write_text(text.rstrip() + append + '\n', encoding='utf-8')
print('OK patched PTZ_MASTER_CONTRACT.md keyboard Q/A movement mode v3')
print('Backup:', backup)