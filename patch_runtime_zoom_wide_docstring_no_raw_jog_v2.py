#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
path = ROOT / 'apply_ptz_object_preset.py'
text = path.read_text(encoding='utf-8', errors='ignore')

marker = 'PTZ_RUNTIME_ZOOM_WIDE_DOCSTRING_NO_RAW_JOG_V2'
if marker in text:
    print('Already patched:', marker)
    raise SystemExit(0)

start = text.find('def zoom_wide_pulse(')
end = text.find('def manual_search_pulse(', start)
if start < 0 or end < 0:
    raise SystemExit('Could not locate zoom_wide_pulse block')

block = text[start:end]
old = 'Never raw /api/zoom/jog here; raw jog can desync physical zoom and PTZ SPEED TUNE sample.'
new = 'PTZ_RUNTIME_ZOOM_WIDE_DOCSTRING_NO_RAW_JOG_V2\n    Never use raw zoom jog here; raw jog can desync physical zoom and PTZ SPEED TUNE sample.'

if old not in block:
    # Fallback: insert marker after contract title without changing logic.
    anchor = '    Wide zoom movement must use the sample contract:\n'
    if anchor not in block:
        raise SystemExit('Could not locate zoom_wide_pulse docstring anchor')
    block2 = block.replace(anchor, anchor + '    PTZ_RUNTIME_ZOOM_WIDE_DOCSTRING_NO_RAW_JOG_V2\n', 1)
else:
    block2 = block.replace(old, new, 1)

# Safety: after patch the scoped executable/doc block must not contain the literal endpoint.
if '/api/zoom/jog' in block2:
    raise SystemExit('Patch would still leave /api/zoom/jog literal in zoom_wide_pulse block; aborting')

backup = path.with_suffix(path.suffix + f'.bak_zoom_wide_docstring_no_raw_jog_v2_{int(time.time())}')
backup.write_text(text, encoding='utf-8')
path.write_text(text[:start] + block2 + text[end:], encoding='utf-8')

s = path.read_text(encoding='utf-8', errors='ignore')
start2 = s.find('def zoom_wide_pulse(')
end2 = s.find('def manual_search_pulse(', start2)
block3 = s[start2:end2]

print('OK patched apply_ptz_object_preset.py zoom_wide docstring no raw jog v2')
print('Backup:', backup)
print('zoom_wide_pulse /api/zoom/jog =', block3.count('/api/zoom/jog'))
print('zoom_wide_pulse /api/zoom/go_to_sample =', block3.count('/api/zoom/go_to_sample'))
print('zoom_wide_pulse apply_ptz_speed_profile_for_zoom =', block3.count('apply_ptz_speed_profile_for_zoom'))
print(marker, '=', s.count(marker))