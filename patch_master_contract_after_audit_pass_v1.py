from pathlib import Path
import time

p = Path("PTZ_MASTER_CONTRACT.md")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_after_audit_pass_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

sections = []

sections.append(r"""
---

# Layer 2 Additional Contract — Persistent Presets Safety

`settings_persist_daemon.py` must never destroy persistent object presets.

## Rule

The daemon is allowed to persist runtime settings to `ui_settings.json` only if runtime settings contain a non-empty:

```text
objectPresetsCustom

If runtime /api/settings temporarily has no objectPresetsCustom, this must be treated as incomplete runtime state.

Correct behavior:

runtime objectPresetsCustom missing/empty
  ↓
settings_persist_daemon.py must skip file write
  ↓
ui_settings.json must keep previous persistent presets

Forbidden behavior:

runtime objectPresetsCustom = {}
  ↓
write ui_settings.json
  ↓
destroy all presets
Required guard
if objectPresetsCustom is missing or empty:
    skip persist
Reason

At service startup or during short-lived runtime transitions /api/settings can briefly contain default/minimal settings.
This transient state must not overwrite the persistent preset database.

Audit expectation
API custom keys != []
FILE custom keys != []
API activeObjectPreset == FILE activeObjectPreset
FILE ptzArmed = false
FILE controlMode = manual

""")

sections.append(r"""
Layer 4 Additional Contract — PTZ Speed Profile Source

/api/autopilot/config has two different classes of fields.

Framing-only fields

These fields are allowed to be sent by object presets:

target_x
target_y
auto_zoom_enable
auto_zoom_target_h
auto_zoom_deadzone
auto_zoom_cmd
auto_zoom_sign
auto_zoom_period_ms

Framing-only config must not change PTZ speed profile source.

Expected:

speed_profile_source remains user/exact/interpolated/clamped_left/clamped_right
speed_profile_source must not become runtime_override
Speed tune fields

Only these fields are allowed to create runtime speed override:

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
Rule

Object preset may send framing-only config.
Object preset must not create PTZ speed override.

Correct behavior
POST /api/autopilot/speed_profile/apply_nearest
  → speed_profile_source = user/exact/interpolated/clamped_*

POST /api/autopilot/config with framing-only fields
  → speed_profile_source must stay unchanged
Forbidden behavior
POST /api/autopilot/config with only target_x/target_y/auto_zoom_*
  → speed_profile_source = runtime_override
Audit expectation
speed_profile_source not in ["", null, "fallback", "runtime_override"]

""")

sections.append(r"""
Layer 5/6 Additional Contract — Daemon vs Runtime Lifetime

Daemon lifecycle audit must distinguish between two events:

daemon child_start observed
runtime process still running after N seconds

These are not the same.

Layer 5 Orchestrator requirement

For a single_auto preset:

ptzArmed=true
  ↓
object_tracking_daemon.py must start apply_ptz_object_preset.py --watch at least once

This proves that the daemon observed the armed state and delegated work to Runtime Strategy.

Layer 6 Runtime requirement

Runtime Strategy may stop itself and set:

ptzArmed=false
controlMode=manual

if the selected search preset is one-shot and no target is acquired.

Therefore this is not a daemon failure:

watch_count = 1 at second 1
watch_count = 0 later

It means:

Daemon started runtime successfully.
Runtime completed or disarmed according to strategy.
Correct daemon audit rule
disarmed → watch_count=0
armed → watch_count observed >=1 during polling window
final disarmed → watch_count=0

Do not require the runtime process to stay alive for one-shot search presets.

Separate runtime strategy audit

Long-running behavior must be tested separately with loop/continuous search preset.

Examples:

lost_wide_cycle
continuous_wide_scan_x

""")

sections.append(r"""
Current Stable Baseline

This baseline was reached after the layer contract cleanup and focused fixes.

Stable audit result
ptz_contract_audit.py
PASS = 62
FAIL = 0
Confirmed fixed items
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
Required command after every layer change
cd /root/new_yolo8
python3 ptz_contract_audit.py

Expected stable result:

FAIL = 0

""")

changed = False

for block in sections:
title = None

for line in block.splitlines():
    if line.startswith("# "):
        title = line.strip("# ").strip()
        break

if title and title in s:
    print("already present:", title)
    continue

s = s.rstrip() + "\n" + block.strip() + "\n"
print("added:", title)
changed = True

if changed:
p.write_text(s, encoding="utf-8")
print("OK updated PTZ_MASTER_CONTRACT.md")
else:
print("NO CHANGES")

print("Backup:", bak)
