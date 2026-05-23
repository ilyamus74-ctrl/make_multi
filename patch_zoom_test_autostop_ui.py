from pathlib import Path
import shutil

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".html.bak_zoom_test_autostop")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

anchor = '''  $('zoomRunCalibrationBtn').onclick = async () => {
    await runZoomCalibrationFromUi();
  };'''

insert = r'''
  let zoomTestStopTimer = null;

  async function zoomTest(cmd, holdMs) {
    cmd = Math.round(Number(cmd || 0));
    holdMs = Math.max(0, Math.min(8000, Math.round(Number(holdMs || 0))));

    if (document.activeElement && typeof document.activeElement.blur === 'function') {
      document.activeElement.blur();
    }

    if (zoomTestStopTimer) {
      clearTimeout(zoomTestStopTimer);
      zoomTestStopTimer = null;
    }

    $('zoomAprilTagCalibStatus').textContent = `test cmd=${cmd} hold=${holdMs}`;

    // Safety stop from browser side.
    if (cmd !== 0 && holdMs > 0) {
      zoomTestStopTimer = setTimeout(() => {
        apiPostJson('/api/zoom_test', { cmd: 0, hold_ms: 0 })
          .then(res => {
            $('zoomAprilTagCalibStatus').textContent = `stopped after ${holdMs}ms`;
            ptzLog('ZOOM TEST autostop', res);
          })
          .catch(e => ptzLog('ZOOM TEST autostop error', { error: String(e.message || e) }));
      }, holdMs + 150);
    }

    const res = await apiPostJson('/api/zoom_test', { cmd, hold_ms: holdMs });
    ptzLog('ZOOM TEST', res);
    return res;
  }

  async function zoomStopNow() {
    if (zoomTestStopTimer) {
      clearTimeout(zoomTestStopTimer);
      zoomTestStopTimer = null;
    }
    if (document.activeElement && typeof document.activeElement.blur === 'function') {
      document.activeElement.blur();
    }
    const res = await apiPostJson('/api/zoom_test', { cmd: 0, hold_ms: 0 });
    $('zoomAprilTagCalibStatus').textContent = 'stopped';
    ptzLog('ZOOM STOP', res);
    return res;
  }

'''

if anchor not in s:
    raise SystemExit("zoomRunCalibrationBtn anchor not found")

if "let zoomTestStopTimer = null;" not in s:
    s = s.replace(anchor, insert + "\n" + anchor, 1)

old = '''  $('zoomStopBtn').onclick = () => {
    apiPostJson('/api/zoom_test', { cmd: 0, hold_ms: 0 })
      .catch(e => ptzLog('ZOOM STOP error', { error: String(e.message || e) }));
  };'''

new = '''  $('zoomStopBtn').onclick = () => {
    zoomStopNow().catch(e => ptzLog('ZOOM STOP error', { error: String(e.message || e) }));
  };'''

if old in s:
    s = s.replace(old, new, 1)
else:
    print("WARN: zoomStopBtn block not found; maybe already patched")

p.write_text(s, encoding="utf-8")
print("patched web/index.html")
