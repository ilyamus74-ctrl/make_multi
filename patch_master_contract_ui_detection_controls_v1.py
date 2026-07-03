#!/usr/bin/env python3
from pathlib import Path
import time

p = Path('PTZ_MASTER_CONTRACT.md')
if not p.exists():
    raise SystemExit('PTZ_MASTER_CONTRACT.md not found')

s = p.read_text(encoding='utf-8', errors='ignore')
bak = p.with_suffix(p.suffix + f'.bak_ui_detection_controls_{int(time.time())}')
bak.write_text(s, encoding='utf-8')

title = 'Layer 1 Additional Contract — Detection Controls Persistence'
block = r'''
---

# Layer 1 Additional Contract — Detection Controls Persistence

Browser UI detection controls are not independent runtime-only fields.

For the active object preset, the following UI controls must persist into the active preset record:

```text
LIMIT      → objectPresetsCustom[activeObjectPreset].max_detections
DETECT FPS → objectPresetsCustom[activeObjectPreset].detect_every_n_frames
AREA       → objectPresetsCustom[activeObjectPreset].detection_mode
```

Top-level settings fields are only a runtime mirror:

```text
operatorDetectionLimit
operatorDetectEvery
operatorDetectionAreaMode
```

## Required save behavior

When the operator changes LIMIT / DETECT FPS / AREA, Browser UI must:

```text
1. read /api/settings
2. resolve activeObjectPreset
3. keep existing objectPresetsCustom, never replace it with an empty object
4. update objectPresetsCustom[activeObjectPreset]
5. update top-level mirror fields
6. POST the full preserved settings object back to /api/settings
7. apply the corresponding detector endpoint
```

## Required hydrate behavior after F5

After page reload, Browser UI must:

```text
1. read /api/settings
2. resolve activeObjectPreset
3. read objectPresetsCustom[activeObjectPreset]
4. set LIMIT / DETECT FPS / AREA controls from that preset
5. paint the active object preset button
```

## Forbidden behavior

Browser UI must not:

```text
POST /api/settings with objectPresetsCustom = {}
restore LIMIT / DETECT FPS / AREA only from localStorage
restore active object preset controls from hardcoded defaults when /api/settings has a saved preset
save LIMIT / DETECT FPS / AREA only as top-level runtime settings
```

## Expected operator result

If the operator selects `car_single`, changes:

```text
LIMIT = 5
AREA = roi
```

then reloads the page and selects `car_single` again, UI must show:

```text
LIMIT = 5
AREA = roi
```

not the canonical fallback values.
'''

if title in s:
    print('already present:', title)
else:
    p.write_text(s.rstrip() + '\n' + block.strip() + '\n', encoding='utf-8')
    print('OK added:', title)
print('Backup:', bak)