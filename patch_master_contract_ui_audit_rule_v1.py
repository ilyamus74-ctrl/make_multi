#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8') if Path('/root/new_yolo8').exists() else Path.cwd()
P = ROOT / 'PTZ_MASTER_CONTRACT.md'

s = P.read_text(encoding='utf-8', errors='ignore')
backup = P.with_suffix(P.suffix + f'.bak_ui_audit_rule_v1_{int(time.time())}')
backup.write_text(s, encoding='utf-8')

title = 'Layer 1 Audit Contract — Detection Controls JavaScript Markers'
block = r'''
---

# Layer 1 Audit Contract — Detection Controls JavaScript Markers

The contract audit must check Browser UI JavaScript markers for detection controls persistence.

## Required behavior

Detection controls are part of the active object preset contract:

```text
LIMIT       → objectPresetsCustom[activeObjectPreset].max_detections
DETECT FPS  → objectPresetsCustom[activeObjectPreset].detect_every_n_frames
AREA        → objectPresetsCustom[activeObjectPreset].detection_mode
```

Top-level settings are only a runtime mirror:

```text
operatorDetectionLimit
operatorDetectEvery
operatorDetectionAreaMode
```

After F5 or after re-selecting the same object preset, Browser UI must hydrate controls from:

```text
/api/settings → objectPresetsCustom[activeObjectPreset]
```

## Required audit checks

`ptz_contract_audit.py` must verify these UI JavaScript markers:

```text
PTZ_CLEAN_DETECTION_UI_HYDRATE_F5_START
PTZ_CLEAN_DETECTION_ACTIVE_PRESET_PERSIST_START
hydrateDetectionControlsFromSettings
scheduleDetectionUiHydrate
persistActiveDetectionControlsToPreset
schedulePersistActiveDetectionControlsToPreset
```

It must also verify that `saveSettings` preserves detection fields and never persists an empty `objectPresetsCustom`.

## Failure meaning

If this layer fails, do not patch Detector, PTZ Autopilot, Runtime Strategy, or Settings daemon first.

The failure belongs to:

```text
Layer 1 Browser UI
```
'''

if title in s:
    print('already present:', title)
else:
    s = s.rstrip() + '\n' + block.strip() + '\n'
    P.write_text(s, encoding='utf-8')
    print('OK added:', title)

print('Backup:', backup)