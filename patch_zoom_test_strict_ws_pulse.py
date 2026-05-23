from pathlib import Path
import shutil

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".html.bak_zoom_test_strict_ws_pulse")
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

  async function sendZoomViaBrowserWsStrict(cmd) {
    cmd = Math.round(Number(cmd || 0));
    cmd = Math.max(-cmdLimits.zoom, Math.min(cmdLimits.zoom, cmd));

    const ok = await ensureWsReady();
    if (!ok) {
      throw new Error('WS not ready for zoom command');
    }

    sendLine(`Z ${cmd}`);
    updateZoomStateFromCommand(cmd);
    $('zoomStick').textContent = `zoom=${cmd} (ZOOM TEST WS)`;

    // Small pause gives ws_uart_bridge time to process the frame.
    await sleepMs(30);

    return cmd;
  }

  async function sendZoomStopBurst(reason = 'stop') {
    // Send several stops. Some firmware/servo states ignore a single short stop.
    for (let i = 0; i < 5; i++) {
      await sendZoomViaBrowserWsStrict(0);
      await sleepMs(70);
    }

    $('zoomAprilTagCalibStatus').textContent = `ws stopped (${reason})`;
    ptzLog('ZOOM TEST WS stop-burst', { cmd: 0, repeat: 5, reason });
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

    $('zoomAprilTagCalibStatus').textContent = `ws pulse cmd=${cmd} hold=${holdMs}`;

    ptzLog('ZOOM TEST WS start', {
      transport: 'browser_ws',
      cmd,
      hold_ms: holdMs
    });

    if (cmd === 0 || holdMs === 0) {
      await sendZoomStopBurst('manual-zero');
      return { ok: true, transport: 'browser_ws', cmd: 0, hold_ms: 0 };
    }

    // 1. Send start and wait until it is actually sent.
    await sendZoomViaBrowserWsStrict(cmd);

    // 2. Only now count hold time.
    await sleepMs(holdMs);

    // 3. If a newer test started, do not fight it.
    if (mySeq !== zoomTestPulseSeq) {
      return { ok: false, transport: 'browser_ws', cancelled: true };
    }

    // 4. Hard stop burst.
    await sendZoomStopBurst(`after ${holdMs}ms`);

    const res = {
      ok: true,
      transport: 'browser_ws',
      cmd,
      hold_ms: holdMs
    };

    ptzLog('ZOOM TEST WS done', res);
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
      transport: 'browser_ws',
      cmd: 0,
      hold_ms: 0
    };

    ptzLog('ZOOM STOP WS', res);
    return res;
  }
'''

s = s[:start] + new_block + s[end:]

p.write_text(s, encoding="utf-8")
print("patched web/index.html")
