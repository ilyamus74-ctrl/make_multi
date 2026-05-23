from pathlib import Path
import shutil

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".html.bak_zoom_test_stream_like_manual")
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
  let zoomTestPulseSeq = 0;

  function sleepMs(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  async function ensureWsReady(timeoutMs = 1200) {
    if (ws && ws.readyState === 1) return true;

    connectWs();

    const started = Date.now();
    while (Date.now() - started < timeoutMs) {
      if (ws && ws.readyState === 1) return true;
      await sleepMs(40);
    }

    return Boolean(ws && ws.readyState === 1);
  }

  async function sendZoomFrame(cmd) {
    cmd = Math.round(Number(cmd || 0));
    cmd = Math.max(-cmdLimits.zoom, Math.min(cmdLimits.zoom, cmd));

    const ok = await ensureWsReady();
    if (!ok) {
      throw new Error('WS not ready for zoom command');
    }

    // Important:
    // Normal manual control sends both J and Z.
    // Some controller firmware may only reliably release zoom when a neutral J frame is also sent.
    seq = (seq + 1) & 0xFFFF;
    sendLine(`J ${seq} 0 0`);
    sendLine(`Z ${cmd}`);

    updateZoomStateFromCommand(cmd);
    $('zoomStick').textContent = `zoom=${cmd} (ZOOM TEST STREAM)`;

    await sleepMs(15);
    return cmd;
  }

  async function sendZoomStopBurst(reason = 'stop') {
    for (let i = 0; i < 8; i++) {
      await sendZoomFrame(0);
      await sleepMs(60);
    }

    $('zoomAprilTagCalibStatus').textContent = `stopped (${reason})`;
    ptzLog('ZOOM TEST stop-burst', { cmd: 0, repeat: 8, reason });
  }

  async function zoomTest(cmd, holdMs) {
    const mySeq = ++zoomTestPulseSeq;

    cmd = Math.round(Number(cmd || 0));
    cmd = Math.max(-cmdLimits.zoom, Math.min(cmdLimits.zoom, cmd));
    holdMs = Math.max(0, Math.min(8000, Math.round(Number(holdMs || 0))));

    if (document.activeElement && typeof document.activeElement.blur === 'function') {
      document.activeElement.blur();
    }

    if (zoomTestStopTimer) {
      clearTimeout(zoomTestStopTimer);
      zoomTestStopTimer = null;
    }

    $('zoomAprilTagCalibStatus').textContent = `stream cmd=${cmd} hold=${holdMs}`;

    ptzLog('ZOOM TEST stream start', {
      transport: 'browser_ws_stream',
      cmd,
      hold_ms: holdMs
    });

    if (cmd === 0 || holdMs === 0) {
      await sendZoomStopBurst('manual-zero');
      return { ok: true, transport: 'browser_ws_stream', cmd: 0, hold_ms: 0 };
    }

    const started = performance.now();

    // Send a continuous command stream during the whole hold time.
    while ((performance.now() - started) < holdMs) {
      if (mySeq !== zoomTestPulseSeq) {
        return { ok: false, transport: 'browser_ws_stream', cancelled: true };
      }

      await sendZoomFrame(cmd);
      await sleepMs(80);
    }

    if (mySeq !== zoomTestPulseSeq) {
      return { ok: false, transport: 'browser_ws_stream', cancelled: true };
    }

    await sendZoomStopBurst(`after ${holdMs}ms`);

    const res = {
      ok: true,
      transport: 'browser_ws_stream',
      cmd,
      hold_ms: holdMs
    };

    ptzLog('ZOOM TEST stream done', res);
    return res;
  }

  async function zoomStopNow() {
    ++zoomTestPulseSeq;

    if (zoomTestStopTimer) {
      clearTimeout(zoomTestStopTimer);
      zoomTestStopTimer = null;
    }

    if (document.activeElement && typeof document.activeElement.blur === 'function') {
      document.activeElement.blur();
    }

    await sendZoomStopBurst('button');

    const res = {
      ok: true,
      transport: 'browser_ws_stream',
      cmd: 0,
      hold_ms: 0
    };

    ptzLog('ZOOM STOP stream', res);
    return res;
  }
'''

s = s[:start] + new_block + s[end:]

p.write_text(s, encoding="utf-8")
print("patched web/index.html")
