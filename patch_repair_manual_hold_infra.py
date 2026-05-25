from pathlib import Path
import shutil
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_repair_manual_hold_infra_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

# 1) Insert backend/manual-hold infrastructure after ptzLog().
marker = """  function panicStop(reason = 'manual') {"""
if marker not in s:
    raise SystemExit("ERROR: panicStop marker not found")

manual_block = r'''
  // Unified backend manual PTZ path.
  // Normal keyboard/touch/gamepad pan/tilt must go through backend.
  let lastManualDriveLogAt = 0;
  let lastManualDriveLogKey = '';

  let manualHoldActive = false;
  let manualHoldPan = 0;
  let manualHoldTilt = 0;
  let manualHoldSource = 'manual';
  let manualHoldTimer = null;
  let manualHoldLastSentAt = 0;

  const MANUAL_HOLD_INTERVAL_MS = 90;
  const MANUAL_HOLD_BACKEND_TTL_MS = 500;

  function maybeLogManualDrive(res) {
    if (!res) return;
    const now = Date.now();
    const key = [
      res.final_pan,
      res.final_tilt,
      res.invert_pan,
      res.invert_tilt,
      res.profile_idx,
      res.speed_profile_source
    ].join('|');

    if (key !== lastManualDriveLogKey || now - lastManualDriveLogAt > 500) {
      ptzLog('MANUAL DRIVE BACKEND', res);
      lastManualDriveLogAt = now;
      lastManualDriveLogKey = key;
    }
  }

  async function backendManualDrive(panNorm, tiltNorm, source = 'manual', holdMs = MANUAL_HOLD_BACKEND_TTL_MS) {
    const res = await apiPostJson(`${autopilotBaseUrl()}/api/control/manual_drive`, {
      pan: Math.max(-1, Math.min(1, Number(panNorm || 0))),
      tilt: Math.max(-1, Math.min(1, Number(tiltNorm || 0))),
      source,
      hold_ms: Math.max(50, Math.min(1000, Number(holdMs || MANUAL_HOLD_BACKEND_TTL_MS)))
    });
    maybeLogManualDrive(res);
    return res;
  }

  async function backendManualStop(source = 'manual_stop') {
    const res = await apiPostJson(`${autopilotBaseUrl()}/api/control/stop`, { source });
    ptzLog('MANUAL STOP BACKEND', res);
    return res;
  }

  async function manualInputDrive(panNorm, tiltNorm, source = 'manual', holdMs = MANUAL_HOLD_BACKEND_TTL_MS) {
    panNorm = Math.max(-1, Math.min(1, Number(panNorm || 0)));
    tiltNorm = Math.max(-1, Math.min(1, Number(tiltNorm || 0)));

    if (Math.abs(panNorm) < 0.001 && Math.abs(tiltNorm) < 0.001) {
      return manualInputStop(source);
    }

    return backendManualDrive(panNorm, tiltNorm, source, holdMs);
  }

  async function manualInputStop(source = 'manual_stop') {
    return backendManualStop(source);
  }

  async function manualHoldTick() {
    if (!manualHoldActive) return;

    const now = Date.now();
    if (now - manualHoldLastSentAt < MANUAL_HOLD_INTERVAL_MS - 5) return;
    manualHoldLastSentAt = now;

    try {
      await manualInputDrive(
        manualHoldPan,
        manualHoldTilt,
        manualHoldSource,
        MANUAL_HOLD_BACKEND_TTL_MS
      );
    } catch (e) {
      ptzLog('MANUAL HOLD error', { error: String(e.message || e) });
    }
  }

  function startManualHold(panNorm, tiltNorm, source = 'manual') {
    panNorm = Math.max(-1, Math.min(1, Number(panNorm || 0)));
    tiltNorm = Math.max(-1, Math.min(1, Number(tiltNorm || 0)));

    manualHoldPan = panNorm;
    manualHoldTilt = tiltNorm;
    manualHoldSource = source;
    manualHoldActive = true;

    if (!manualHoldTimer) {
      manualHoldTimer = setInterval(manualHoldTick, MANUAL_HOLD_INTERVAL_MS);
    }

    manualHoldTick();
  }

  async function stopManualHold(source = 'manual_stop') {
    manualHoldActive = false;
    manualHoldPan = 0;
    manualHoldTilt = 0;

    if (manualHoldTimer) {
      clearInterval(manualHoldTimer);
      manualHoldTimer = null;
    }

    try {
      await manualInputStop(source);
    } catch (e) {
      ptzLog('MANUAL HOLD STOP error', { error: String(e.message || e) });
    }
  }

  function directionForTouchCmd(cmd) {
    if (cmd === 'left') return [-1, 0];
    if (cmd === 'right') return [1, 0];
    if (cmd === 'up') return [0, -1];
    if (cmd === 'down') return [0, 1];
    return [0, 0];
  }

  function installUnifiedManualHoldControls() {
    if (window.__manualHoldControlsInstalled) return;
    window.__manualHoldControlsInstalled = true;

    const manualKeyState = new Set();

    window.addEventListener('keydown', (ev) => {
      if (isTypingTarget(ev.target)) return;

      const k = ev.key;
      if (!['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(k)) return;

      ev.preventDefault();

      if (manualKeyState.has(k)) return;
      manualKeyState.add(k);

      if (k === 'ArrowLeft') startManualHold(-1, 0, 'keyboard_left');
      if (k === 'ArrowRight') startManualHold(1, 0, 'keyboard_right');
      if (k === 'ArrowUp') startManualHold(0, -1, 'keyboard_up');
      if (k === 'ArrowDown') startManualHold(0, 1, 'keyboard_down');
    }, true);

    window.addEventListener('keyup', (ev) => {
      const k = ev.key;
      if (!['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(k)) return;

      manualKeyState.delete(k);

      if (manualKeyState.size === 0) {
        stopManualHold('keyboard_keyup');
      }
    }, true);

    window.addEventListener('blur', () => {
      manualKeyState.clear();
      stopManualHold('window_blur');
    });

    document.querySelectorAll('.touch-btn[data-cmd]').forEach(btn => {
      const cmd = btn.dataset.cmd;
      if (!['up', 'down', 'left', 'right'].includes(cmd)) return;

      btn.addEventListener('pointerdown', (ev) => {
        ev.preventDefault();
        const [pan, tilt] = directionForTouchCmd(cmd);
        startManualHold(pan, tilt, `touch_${cmd}`);
      });

      ['pointerup', 'pointercancel', 'pointerleave'].forEach(evt => {
        btn.addEventListener(evt, (ev) => {
          ev.preventDefault();
          stopManualHold(`touch_${cmd}_stop`);
        });
      });
    });
  }

'''

if "function startManualHold(" not in s:
    s = s.replace(marker, manual_block + "\n" + marker, 1)
    print("OK: inserted manual hold infrastructure")
else:
    print("SKIP: manual hold infrastructure already exists")

# 2) Replace old panicStop manualDrive call.
old = "    manualDrive(0, 0, 0);"
new = "    manualInputStop('panic_stop').catch(() => {});"
if old in s:
    s = s.replace(old, new, 1)
    print("OK: replaced old manualDrive(0,0,0) in panicStop")
else:
    print("SKIP: old manualDrive(0,0,0) not found")

# 3) Install controls before initSettings().
call = "  installUnifiedManualHoldControls();\n"
target = "  initSettings();"
if "installUnifiedManualHoldControls();" not in s:
    if target not in s:
      raise SystemExit("ERROR: initSettings call not found")
    s = s.replace(target, call + target, 1)
    print("OK: installed manual hold controls before initSettings")
else:
    print("SKIP: installUnifiedManualHoldControls already called")

p.write_text(s, encoding="utf-8")
print("DONE")
