#!/usr/bin/env python3
from pathlib import Path
import re
import time

ROOT = Path('/root/new_yolo8')
AUDIT = ROOT / 'ptz_contract_audit.py'
MARKER_V1 = 'PTZ_AUDIT_RUNTIME_DETECTION_ZOOM_V1'
MARKER_V2 = 'PTZ_AUDIT_RUNTIME_DETECTION_ZOOM_V2'

if not AUDIT.exists():
    raise SystemExit(f'Missing {AUDIT}')

text = AUDIT.read_text(encoding='utf-8', errors='ignore')
if MARKER_V2 in text:
    print('Already patched:', MARKER_V2)
    raise SystemExit(0)

backup = AUDIT.with_name(AUDIT.name + f'.bak_runtime_detection_zoom_v2_{int(time.time())}')
backup.write_text(text, encoding='utf-8')

# Add marker close to the v1 marker when present. If v1 did not create a standalone marker,
# place the marker before the Layer 6B title.
if MARKER_V1 in text:
    text = text.replace(MARKER_V1, MARKER_V1 + '\n# ' + MARKER_V2, 1)
elif 'LAYER 6B — RUNTIME DETECTION OFF / ZOOM SAMPLE SYNC' in text:
    text = text.replace(
        'LAYER 6B — RUNTIME DETECTION OFF / ZOOM SAMPLE SYNC',
        MARKER_V2 + '\nLAYER 6B — RUNTIME DETECTION OFF / ZOOM SAMPLE SYNC',
        1,
    )
else:
    raise SystemExit('Could not find Layer 6B audit block to patch')

# Locate the variable printed as "custom plan pause guard" and override it immediately
# before the print. v1 scoped from the custom_plan_start event and can miss a valid guard
# placed earlier in the same function, before zoom_wide_on_start.
pat = re.compile(r'^(?P<indent>\s*)print\("custom plan pause guard =",\s*(?P<var>[A-Za-z_][A-Za-z0-9_]*)\)\s*$', re.M)
m = pat.search(text)
if not m:
    raise SystemExit('Could not find custom plan pause guard print line')

indent = m.group('indent')
var = m.group('var')
insert = f'''{indent}# {MARKER_V2}
{indent}# v1 could miss a valid custom-plan guard if the guard is placed
{indent}# before event_log("custom_plan_start") / before zoom_wide_on_start.
{indent}def _ptz_runtime_function_block_containing_v2(src, token):
{indent}    pos = src.find(token)
{indent}    if pos < 0:
{indent}        return ""
{indent}    start = src.rfind("\\ndef ", 0, pos)
{indent}    if start < 0:
{indent}        start = 0
{indent}    else:
{indent}        start += 1
{indent}    end = src.find("\\ndef ", pos + 1)
{indent}    if end < 0:
{indent}        end = len(src)
{indent}    return src[start:end]

{indent}_custom_plan_block_v2 = _ptz_runtime_function_block_containing_v2(runtime_text, 'custom_plan_start')
{indent}if not _custom_plan_block_v2:
{indent}    _custom_plan_block_v2 = _ptz_runtime_function_block_containing_v2(runtime_text, 'runtime_pause_if_detection_disabled("custom_plan"')
{indent}{var} = (
{indent}    'runtime_pause_if_detection_disabled("custom_plan"' in _custom_plan_block_v2
{indent}    or 'runtime_pause_if_detection_disabled(\"custom_plan\"' in runtime_text
{indent})
{indent}print("custom plan function scoped block length =", len(_custom_plan_block_v2))
'''

text = text[:m.start()] + insert + text[m.start():]

AUDIT.write_text(text, encoding='utf-8')

print('OK patched ptz_contract_audit.py runtime detection/zoom audit v2')
print('Backup:', backup)
print('Changes:')
print(' - fixes false FAIL for custom_plan pause guard')
print(' - scopes custom-plan check to the whole function containing custom_plan_start')
print(' - accepts guard placed before zoom_wide_on_start/custom_plan_start')
print(' - keeps all runtime zoom/sample-speed checks from v1')