#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Patch PTZ_MASTER_CONTRACT.md after stable audit baseline.

Usage:
  cd /root/new_yolo8
  python3 patch_master_contract_after_audit_pass_v2.py

This script:
  - makes a timestamped backup of PTZ_MASTER_CONTRACT.md
  - appends missing contract sections only once
  - does not remove or rewrite existing sections
"""

from pathlib import Path
import time
import sys

ROOT = Path(".")
CONTRACT = ROOT / "PTZ_MASTER_CONTRACT.md"

SECTIONS = [
    {
        "title": "Layer 2 Additional Contract — Persistent Presets Safety",
        "body": """---

# Layer 2 Additional Contract — Persistent Presets Safety

`settings_persist_daemon.py` must never destroy persistent object presets.

## Rule

The daemon is allowed to persist runtime settings to `ui_settings.json` only if runtime settings contain a non-empty:

```text
objectPresetsCustom
```

If runtime `/api/settings` temporarily has no `objectPresetsCustom`, this must be treated as incomplete runtime state.

Correct behavior:

```text
runtime objectPresetsCustom missing/empty
  ↓
settings_persist_daemon.py must skip file write
  ↓
ui_settings.json must keep previous persistent presets
```

Forbidden behavior:

```text
runtime objectPresetsCustom = {}
  ↓
write ui_settings.json
  ↓
destroy all presets
```

## Required guard

```text
if objectPresetsCustom is missing or empty:
    skip persist
```

## Reason

At service startup or during short-lived runtime transitions `/api/settings` can briefly contain default/minimal settings.

This transient state must not overwrite the persistent preset database.

## Audit expectation

```text
API custom keys != []
FILE custom keys != []
API activeObjectPreset == FILE activeObjectPreset
FILE ptzArmed = false
FILE controlMode = manual
```
""",
    },
    {
        "title": "Layer 4 Additional Contract — PTZ Speed Profile Source",
        "body": """---

# Layer 4 Additional Contract — PTZ Speed Profile Source

`/api/autopilot/config` has two different classes of fields.

## Framing-only fields

These fields are allowed to be sent by object presets:

```text
target_x
target_y
auto_zoom_enable
auto_zoom_target_h
auto_zoom_deadzone
auto_zoom_cmd
auto_zoom_sign
auto_zoom_period_ms
```

Framing-only config must not change PTZ speed profile source.

Expected:

```text
speed_profile_source remains user/exact/interpolated/clamped_left/clamped_right
speed_profile_source must not become runtime_override
```

## Speed tune fields

Only these fields are allowed to create runtime speed override:

```text
kp
ki
kd
deadzone
max_pan
max_tilt
max_accel
min_pan
min_tilt
hz
```

## Rule

Object preset may send framing-only config.

Object preset must not create PTZ speed override.

## Correct behavior

```text
POST /api/autopilot/speed_profile/apply_nearest
  → speed_profile_source = user/exact/interpolated/clamped_*

POST /api/autopilot/config with framing-only fields
  → speed_profile_source must stay unchanged
```

## Forbidden behavior

```text
POST /api/autopilot/config with only target_x/target_y/auto_zoom_*
  → speed_profile_source = runtime_override
```

## Audit expectation

```text
speed_profile_source not in ["", null, "fallback", "runtime_override"]
```
""",
    },
    {
        "title": "Layer 5/6 Additional Contract — Daemon vs Runtime Lifetime",
        "body": """---

# Layer 5/6 Additional Contract — Daemon vs Runtime Lifetime

Daemon lifecycle audit must distinguish between two events:

```text
daemon child_start observed
runtime process still running after N seconds
```

These are not the same.

## Layer 5 Orchestrator requirement

For a `single_auto` preset:

```text
ptzArmed=true
  ↓
object_tracking_daemon.py must start apply_ptz_object_preset.py --watch at least once
```

This proves that the daemon observed the armed state and delegated work to Runtime Strategy.

## Layer 6 Runtime requirement

Runtime Strategy may stop itself and set:

```text
ptzArmed=false
controlMode=manual
```

if the selected search preset is one-shot and no target is acquired.

Therefore this is not a daemon failure:

```text
watch_count = 1 at second 1
watch_count = 0 later
```

It means:

```text
Daemon started runtime successfully.
Runtime completed or disarmed according to strategy.
```

## Correct daemon audit rule

```text
disarmed → watch_count=0
armed → watch_count observed >=1 during polling window
final disarmed → watch_count=0
```

Do not require the runtime process to stay alive for one-shot search presets.

## Separate runtime strategy audit

Long-running behavior must be tested separately with loop/continuous search preset.

Examples:

```text
lost_wide_cycle
continuous_wide_scan_x
```
""",
    },
    {
        "title": "Current Stable Baseline",
        "body": """---

# Current Stable Baseline

This baseline was reached after the layer contract cleanup and focused fixes.

## Stable audit result

```text
ptz_contract_audit.py
PASS = 62
FAIL = 0
```

## Confirmed fixed items

```text
Layer 1 Browser UI:
  clean controller active
  old object preset handlers removed
  object preset click does not auto-arm PTZ

Layer 2 Settings:
  API and FILE objectPresetsCustom are non-empty
  settings_persist_daemon.py does not persist empty objectPresetsCustom
  active preset exists in objectPresetsCustom
  FILE ptzArmed=false
  FILE controlMode=manual

Layer 3 Detector:
  limits/throttle/roi/classes match active object preset

Layer 4 PTZ:
  framing-only /api/autopilot/config does not create runtime_override
  speed_profile_source remains user after framing-only config

Layer 5 Orchestrator:
  daemon starts runtime child when ptzArmed=true and preset is single_auto

Layer 6 Runtime:
  runtime may stop itself for one-shot search preset
  this is not a daemon failure
```

## Required command after every layer change

```bash
cd /root/new_yolo8
python3 ptz_contract_audit.py
```

Expected stable result:

```text
FAIL = 0
```
""",
    },
]


def main() -> int:
    if not CONTRACT.exists():
        print(f"ERROR: {CONTRACT} not found")
        print("Run this script from /root/new_yolo8")
        return 1

    text = CONTRACT.read_text(encoding="utf-8", errors="ignore")

    backup = CONTRACT.with_suffix(CONTRACT.suffix + f".bak_after_audit_pass_{int(time.time())}")
    backup.write_text(text, encoding="utf-8")

    added = []
    skipped = []

    for section in SECTIONS:
        title = section["title"]
        body = section["body"].strip() + "\n"

        if title in text:
            skipped.append(title)
            continue

        text = text.rstrip() + "\n\n" + body
        added.append(title)

    if added:
        CONTRACT.write_text(text, encoding="utf-8")

    print("Backup:", backup)
    print("Added sections:", len(added))
    for title in added:
        print("  +", title)

    print("Skipped sections:", len(skipped))
    for title in skipped:
        print("  =", title)

    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())