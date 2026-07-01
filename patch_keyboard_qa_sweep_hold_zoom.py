from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_keyboard_qa_sweep_hold_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

block = r'''
<script>
// KEYBOARD_QA_SWEEP_HOLD_ZOOM_START
(function () {
  if (window.__keyboardQaSweepHoldZoomInstalled) return;
  window.__keyboardQaSweepHoldZoomInstalled = true;

  const $ = (id) => document.getElementById(id);

  let active = null;

  function isTypingTarget(el) {
    if (!el) return false;
    const tag = String(el.tagName || '').toLowerCase();
    return tag === 'input' || tag === 'textarea' || tag === 'select' || Boolean(el.isContentEditable);
  }

  function isSweepMode() {
    const v = $('zoomMoveMode')?.value || '';
    return v === 'sweep_time_steps';
  }

  function cmdAbs() {
    return Math.max(1, Math.min(100, Number($('zoomCmdAbs')?.value || 34)));
  }

  function wideSign() {
    return Number($('zoomWideSign')?.value || 1) < 0 ? -1 : 1;
  }

  function wideCmd() {
    return wideSign() * cmdAbs();
  }

  function teleCmd() {
    return -wideCmd();
  }

  function fullSweepMs() {
    return Math.max(100, Number($('zoomFullSweepMs')?.value || $('zoomWideHoldMs')?.value || 1500));
  }

  function sampleCount() {
    return Math.max(2, Math.min(64, Number($('zoomSamples')?.value || 10)));
  }

  function stepMs() {
    return fullSweepMs() / Math.max(1, sampleCount() - 1);
  }

  async function getJson(url) {
    const r = await fetch(url + (url.includes('?') ? '&' : '?') + 'ts=' + Date.now(), { cache: 'no-store' });
    if (!r.ok) throw new Error(`GET ${url} HTTP ${r.status}`);
    return await r.json();
  }

  async function postJson(url, payload) {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload || {})
    });
    if (!r.ok) throw new Error(`POST ${url} HTTP ${r.status}`);
    return await r.json().catch(() => ({ ok: true }));
  }

  function sendZoomCmd(cmd) {
    cmd = Math.round(Number(cmd || 0));

    try {
      if (typeof window.sendZoomOnly === 'function') {
        window.sendZoomOnly(cmd);
        return true;
      }
    } catch (_) {}

    try {
      if (typeof window.sendLine === 'function') {
        window.sendLine(`Z ${cmd}`);
        return true;
      }
    } catch (_) {}

    try {
      if (window.ws && window.ws.readyState === 1) {
        window.ws.send(`Z ${cmd}\n`);
        return true;
      }
    } catch (_) {}

    console.warn('[keyboard-qa-sweep] cannot send zoom cmd', cmd);
    return false;
  }

  async function loadCurrentSample() {
    try {
      const zs = await getJson('/api/zoom/state');
      const idx = Number(zs.zoom_sample_idx || 0);
      const count = Number(zs.zoom_sample_count || sampleCount());
      return {
        idx: Number.isFinite(idx) ? idx : 0,
        count: Number.isFinite(count) && count > 0 ? count : sampleCount(),
        ratio: Number(zs.zoom_ratio || 0)
      };
    } catch (_) {
      return { idx: 0, count: sampleCount(), ratio: 0 };
    }
  }

  async function updateBackendSampleEstimate(direction, heldMs, startIdx) {
    const count = sampleCount();
    const maxIdx = count - 1;

    const rawDelta = heldMs / Math.max(1, stepMs());
    const delta = Math.max(0, Math.round(rawDelta));
    const target = Math.max(0, Math.min(maxIdx, Number(startIdx || 0) + direction * delta));
    const ratio = maxIdx > 0 ? target / maxIdx : 0;

    const payload = {
      zoom_sample_idx: target,
      zoom_ratio: ratio,
      zoom_confidence: 0.75,
      zoom_source: 'keyboard_qa_sweep_hold'
    };

    try {
      await postJson('/api/zoom/state', payload);
    } catch (e) {
      console.warn('[keyboard-qa-sweep] zoom state update failed', e);
    }

    try {
      if (typeof window.ptzTuneCurrentSample !== 'undefined') {
        window.ptzTuneCurrentSample = target;
      }
      await window.ptzTuneLoadSpeedForSample?.(target);
      window.updatePtzTuneSampleButtonUi?.();
    } catch (_) {}

    try {
      window.ptzLog?.('KEYBOARD QA SWEEP HOLD DONE', {
        start_idx: startIdx,
        target_idx: target,
        held_ms: Math.round(heldMs),
        step_ms: Number(stepMs().toFixed(1)),
        raw_delta: Number(rawDelta.toFixed(2)),
        delta,
        direction
      });
    } catch (_) {}

    if ($('ptzTuneStatus')) {
      $('ptzTuneStatus').textContent =
        `Q/A sweep hold: ${startIdx}→${target}, held=${Math.round(heldMs)}ms step=${stepMs().toFixed(1)}ms`;
    }
  }

  async function startSweepKey(e, isQ) {
    if (active) return;

    const st = await loadCurrentSample();
    const direction = isQ ? 1 : -1; // Q=TELE, A=WIDE
    const cmd = isQ ? teleCmd() : wideCmd();

    active = {
      key: isQ ? 'q' : 'a',
      direction,
      cmd,
      startTs: performance.now(),
      startIdx: st.idx
    };

    sendZoomCmd(cmd);

    try {
      window.ptzLog?.('KEYBOARD QA SWEEP HOLD START', {
        key: active.key,
        cmd,
        start_idx: st.idx,
        step_ms: Number(stepMs().toFixed(1)),
        full_sweep_ms: fullSweepMs(),
        samples: sampleCount()
      });
    } catch (_) {}

    if ($('ptzTuneStatus')) {
      $('ptzTuneStatus').textContent =
        `Q/A sweep hold START key=${active.key.toUpperCase()} cmd=${cmd} start=${st.idx}`;
    }
  }

  async function stopSweepKey(reason = 'keyup') {
    if (!active) return;

    const a = active;
    active = null;

    sendZoomCmd(0);

    const heldMs = Math.max(0, performance.now() - a.startTs);
    await updateBackendSampleEstimate(a.direction, heldMs, a.startIdx);

    try {
      window.ptzLog?.('KEYBOARD QA SWEEP HOLD STOP', {
        reason,
        key: a.key,
        held_ms: Math.round(heldMs)
      });
    } catch (_) {}
  }

  window.addEventListener('keydown', (e) => {
    const key = String(e.key || '').toLowerCase();
    const code = String(e.code || '');
    const isQ = key === 'q' || code === 'KeyQ';
    const isA = key === 'a' || code === 'KeyA';

    if (!isQ && !isA) return;
    if (isTypingTarget(e.target)) return;
    if (!isSweepMode()) return;

    e.preventDefault();
    e.stopPropagation();
    e.stopImmediatePropagation();

    if (e.repeat) return;

    startSweepKey(e, isQ).catch(err => {
      console.error('[keyboard-qa-sweep] start failed', err);
      sendZoomCmd(0);
      active = null;
    });
  }, true);

  window.addEventListener('keyup', (e) => {
    const key = String(e.key || '').toLowerCase();
    const code = String(e.code || '');
    const isQ = key === 'q' || code === 'KeyQ';
    const isA = key === 'a' || code === 'KeyA';

    if (!isQ && !isA) return;
    if (!active) return;
    if (!isSweepMode()) return;

    e.preventDefault();
    e.stopPropagation();
    e.stopImmediatePropagation();

    stopSweepKey('keyup').catch(err => {
      console.error('[keyboard-qa-sweep] stop failed', err);
      sendZoomCmd(0);
      active = null;
    });
  }, true);

  window.addEventListener('blur', () => {
    if (active) stopSweepKey('window_blur').catch(() => sendZoomCmd(0));
  });

  document.addEventListener('visibilitychange', () => {
    if (document.hidden && active) stopSweepKey('visibility_hidden').catch(() => sendZoomCmd(0));
  });

  window.keyboardQaSweepHoldStop = () => stopSweepKey('manual_console');

  console.log('[keyboard-qa-sweep] installed');
})();
// KEYBOARD_QA_SWEEP_HOLD_ZOOM_END
</script>
'''

if "KEYBOARD_QA_SWEEP_HOLD_ZOOM_START" in s:
    print("SKIP: keyboard Q/A sweep hold already installed")
else:
    if "</body>" in s:
        s = s.replace("</body>", block + "\n</body>", 1)
    elif "</html>" in s:
        s = s.replace("</html>", block + "\n</html>", 1)
    else:
        s += "\n" + block + "\n"

    p.write_text(s, encoding="utf-8")
    print("OK: installed Q/A sweep hold keyboard handler")
