from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_qa_use_backend_zoom_jog_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

changed = False

old_start = """    sendZoomOnly(cmd);

    ptzLog('KEYBOARD QA SWEEP HOLD START', {
"""

new_start = """    const jogRes = await apiPostJson('/api/zoom/jog', {
      action: 'start',
      cmd,
      hold_ms: keyboardQaFullSweepMs(),
      source
    });

    ptzLog('KEYBOARD QA SWEEP JOG START', jogRes);

    ptzLog('KEYBOARD QA SWEEP HOLD START', {
"""

if "KEYBOARD QA SWEEP JOG START" not in s:
    if old_start not in s:
        raise SystemExit("ERROR: Q/A sweep start sendZoomOnly block not found")
    s = s.replace(old_start, new_start, 1)
    print("OK: Q/A sweep start now uses backend /api/zoom/jog")
    changed = True
else:
    print("SKIP: start already uses backend jog")

old_stop = """    sendZoomOnly(0);

    const heldMs = Math.max(0, performance.now() - h.startTs);
"""

new_stop = """    const jogStopRes = await apiPostJson('/api/zoom/jog', {
      action: 'stop',
      cmd: 0,
      source: 'keyboard_qa_sweep_hold_stop'
    });

    ptzLog('KEYBOARD QA SWEEP JOG STOP', jogStopRes);

    const heldMs = Math.max(0, performance.now() - h.startTs);
"""

if "KEYBOARD QA SWEEP JOG STOP" not in s:
    if old_stop not in s:
        raise SystemExit("ERROR: Q/A sweep stop sendZoomOnly block not found")
    s = s.replace(old_stop, new_stop, 1)
    print("OK: Q/A sweep stop now uses backend /api/zoom/jog stop")
    changed = True
else:
    print("SKIP: stop already uses backend jog")

# Error handlers should stop via backend too, but keep sendZoomOnly fallback.
s = s.replace(
    "sendZoomOnly(0);\n            keyboardQaSweepHold = null;",
    "apiPostJson('/api/zoom/jog', { action: 'stop', cmd: 0 }).catch(() => sendZoomOnly(0));\n            keyboardQaSweepHold = null;"
)

if changed:
    p.write_text(s, encoding="utf-8")
    print("DONE")
else:
    print("No changes")
