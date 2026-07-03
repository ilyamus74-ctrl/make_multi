#!/usr/bin/env python3
from pathlib import Path
import time

p = Path('/root/new_yolo8/PTZ_MASTER_CONTRACT.md')
s = p.read_text(encoding='utf-8', errors='ignore')

heading = '## Layer 1C Contract — Keyboard Q/A Sweep'
if heading in s:
    print('Already patched PTZ_MASTER_CONTRACT.md keyboard Q/A sweep contract v1')
    raise SystemExit(0)

bak = p.with_name(p.name + f'.bak_keyboard_qa_sweep_v1_{int(time.time())}')
bak.write_text(s, encoding='utf-8')

block = f'''

{heading}

Q/A keyboard controls are part of the Browser UI layer, but they must command the backend, not only edit DOM fields.

Required behavior:

- `Q` means TELE / zoom sample `+1`.
- `A` means WIDE / zoom sample `-1`.
- In `SWEEP TIME STEPS` mode, Q/A must call `/api/zoom/go_to_sample` with `profile_idx`, not `sample_idx`.
- After the backend zoom move, Q/A must apply PTZ SPEED TUNE for the same sample using `/api/autopilot/speed_profile/apply_nearest` with `profile_idx`.
- After the backend confirms the move, UI must refresh `ptzTuneCurrentSample`, PTZ SPEED TUNE fields, and sample button state.

Forbidden behavior:

- Q/A must not only update `ptzTuneCurrentSample` or `/api/zoom/state` without moving the backend zoom.
- Q/A must not use `/api/zoom/go_to_sample` with `sample_idx`; the canonical request key is `profile_idx`.
- Q/A must not let older duplicate keyboard handlers consume the event before the canonical handler.

Audit marker:

- `PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_START`

'''

s = s.rstrip() + block
p.write_text(s + '\n', encoding='utf-8')
print('OK patched PTZ_MASTER_CONTRACT.md keyboard Q/A sweep contract v1')
print('Backup:', bak)