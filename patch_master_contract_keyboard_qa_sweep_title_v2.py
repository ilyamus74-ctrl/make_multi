#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
MD = ROOT / 'PTZ_MASTER_CONTRACT.md'
TITLE = 'Layer 1C Additional Contract — Keyboard Q/A Sweep'

if not MD.exists():
    raise SystemExit(f'Missing {MD}')

text = MD.read_text(encoding='utf-8', errors='ignore')
if TITLE in text:
    print('Already present:', TITLE)
    raise SystemExit(0)

backup = MD.with_name(MD.name + f'.bak_keyboard_qa_sweep_title_v2_{int(time.time())}')
backup.write_text(text, encoding='utf-8')

append = f'''

## {TITLE}

Q/A keyboard controls in `SWEEP TIME STEPS` mode must be owned by a high-priority capture handler.

Required behavior:

- `Q` moves the real backend zoom sample to `profile_idx + 1`.
- `A` moves the real backend zoom sample to `profile_idx - 1`.
- The backend request must use `/api/zoom/go_to_sample` with `profile_idx`, not `sample_idx`.
- After zoom movement, PTZ speed tune must be applied using `/api/autopilot/speed_profile/apply_nearest` with `profile_idx`.
- The handler must call `preventDefault`, `stopPropagation`, and `stopImmediatePropagation` before older Q/A handlers can run.
- UI sample buttons and PTZ SPEED TUNE fields must be refreshed from backend state after movement.

Audit coverage:

- `LAYER 1C — KEYBOARD Q/A SWEEP JS CONTRACT`
- marker `PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_START`
- marker `PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_END`
'''

MD.write_text(text.rstrip() + append + '\n', encoding='utf-8')
print('OK patched PTZ_MASTER_CONTRACT.md keyboard Q/A sweep title v2')
print('Backup:', backup)