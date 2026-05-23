from pathlib import Path
import shutil

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".html.bak_zoom_test_browser_ws")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

start = s.find("  let zoomTestStopTimer = null;")
if start < 0:
    raise SystemExit("zoomTestStopTimer block start not found")

end = s.find("\n\n  $('zoomRunCalibrationBtn').onclick", start)
if end < 0:
    raise SystemExit("zoom test block end not found")

new_block = r'''  let zoomTestStopTimer = null;

  function runWhenWsReady(fn) {
    if (ws && ws.readyState === 1) {
      fn();
      return;
    }

    connectWs();

    let done = false;
    const tryRun = () => {
      if (done) return;
      if (ws && ws.readyState === 1) {
        done = true;
        fn();
      }
    };

    setTimeout(tryRun, 80);
    setTimeout(tryRun, 200);
    setTimeout(tryRun, 500);
  }

  function sendZoomViaBrowserWs(cmd) {
    cmd = Math.round(Number(cmd || 0));
    cmd = Math.max(-cmdLimits.zoom, Math.min(cmdLimits.zoom, cmd));

    runWhenWsReady(() => {
      sendLine(`Z ${cmd}`);
      updateZoomStateFromCommand(cmd);
      $('zoomStick').textContent = `zoom=${cmd} (ZOOM TEST WS)`;
    });
  }

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

    $('zoomAprilTagCalibStatus').textContent = `ws test cmd=${cmd} hold=${holdMs}`;

    sendZoomViaBrowserWs(cmd);

    if (cmd !== 0 && holdMs > 0) {
      zoomTestStopTimer = setTimeout(() => {
        sendZoomViaBrowserWs(0);
        $('zoomAprilTagCalibStatus').textContent = `ws stopped after ${holdMs}ms`;
        ptzLog('ZOOM TEST WS autostop', { cmd: 0, hold_ms: 0 });
      }, holdMs);
    }

    const res = {
      ok: true,
      transport: 'browser_ws',
      cmd,
      hold_ms: holdMs
    };

    ptzLog('ZOOM TEST WS', res);
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

    sendZoomViaBrowserWs(0);

    const res = {
      ok: true,
      transport: 'browser_ws',
      cmd: 0,
      hold_ms: 0
    };

    $('zoomAprilTagCalibStatus').textContent = 'ws stopped';
    ptzLog('ZOOM STOP WS', res);
    return res;
  }
'''

s = s[:start] + new_block + s[end:]

p.write_text(s, encoding="utf-8")
print("patched web/index.html")
