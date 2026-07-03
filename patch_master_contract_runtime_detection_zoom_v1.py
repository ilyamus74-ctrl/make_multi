#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
MD = ROOT / 'PTZ_MASTER_CONTRACT.md'
TITLE = 'Layer 6B Additional Contract — Runtime Detection OFF / Zoom Sample Sync'
MARKER = 'PTZ_MASTER_CONTRACT_RUNTIME_DETECTION_ZOOM_V1'

if not MD.exists():
    raise SystemExit(f'Missing {MD}')

text = MD.read_text(encoding='utf-8', errors='ignore')
if MARKER in text or TITLE in text:
    print('Already present:', TITLE)
    raise SystemExit(0)

backup = MD.with_name(MD.name + f'.bak_runtime_detection_zoom_v1_{int(time.time())}')
backup.write_text(text, encoding='utf-8')

append = f'''

## {TITLE}

<!-- {MARKER} -->

Runtime strategy code in `apply_ptz_object_preset.py` must respect the operator's detection state and the zoom-sample speed contract.

Required behavior:

- If the operator sets detector `detect_enabled=false` while PTZ/runtime is active, runtime must pause/stop tracking and search behavior.
- Detection OFF must not start object reacquire/search.
- Detection OFF must not move pan/tilt or zoom as part of lost-object recovery.
- Runtime state should report a paused/no-detection condition, for example `paused_detection_off` or equivalent event/state naming.
- Runtime search may resume only after detection is explicitly enabled again and the normal arm/runtime loop permits it.

Zoom movement contract:

- Any real runtime zoom movement must use `/api/zoom/go_to_sample` with `profile_idx`.
- Runtime zoom movement must not use raw `/api/zoom/jog` for search/wide/reacquire movement.
- After every runtime zoom sample move, runtime must apply PTZ speed tuning through `/api/autopilot/speed_profile/apply_nearest` using the resulting `profile_idx` / sample.
- `zoom_wide_pulse()` is part of this contract: it must move wide by sample/profile, not by raw jog.

Audit coverage:

- `LAYER 6B — RUNTIME DETECTION OFF / ZOOM SAMPLE SYNC`
- marker `PTZ_RUNTIME_DETECTION_OFF_PAUSE_GUARD_V1_START`
- marker `PTZ_RUNTIME_DETECTION_OFF_PAUSE_GUARD_V1_END`
- marker `PTZ_RUNTIME_ZOOM_SAMPLE_SPEED_SYNC_V1`
- marker `PTZ_RUNTIME_ZOOM_WIDE_DOCSTRING_NO_RAW_JOG_V2`
'''

MD.write_text(text.rstrip() + append + '\n', encoding='utf-8')

print('OK patched PTZ_MASTER_CONTRACT.md runtime detection/zoom contract v1')
print('Backup:', backup)
print('Changes:')
print(' - added Layer 6B runtime detection-off / zoom sample sync contract')
print(' - documents Detection OFF pause behavior')
print(' - documents runtime zoom go_to_sample + apply_nearest contract')