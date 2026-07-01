from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_qa_sweep_backend_jog_hard_override_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

block = r'''
<script>
// QA_SWEEP_BACKEND_JOG_HARD_OVERRIDE_START
(function () {
  if (window.__qaSweepBackendJogHardOverrideInstalled) return;
  window.__qaSweepBackendJogHardOverrideInstalled = true;

  const $ = (id) => document.getElementById(id);
  let active = null;

  function isSweep() {
    return ($('zoomMoveMode')?.value || '') === 'sweep_time_steps';
  }

  function samples() {
    return Math.max(2, Math.min(100, Number($('zoomSamples')?.value || 10)));
  }

  function fullSweepMs() {
    return Math.max(100, Math.min(20000, Number($('zoomFullSweepMs')?.value || $('zoomWideHoldMs')?.value || 1500)));
  }

  function stepMs() {
    return fullSweepMs() / Math.max(1, samples() - 1);
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

  async function postJson(url, payload) {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      cache: 'no-store',
      body: JSON.stringify(payload || {})
    });
    const text = await r.text();
    let data = {};
    try { data = text ? JSON.parse(text) : {}; } catch (_) { data = { raw: text }; }
    if (!r.ok) throw new Error(`${url} HTTP ${r.status}: ${text}`);
    return data;
  }

  async function getJson(url) {
    const r = await fetch(`${url}${url.includes('?') ? '&' : '?'}ts=${Date.now()}`, {
      cache: 'no-store'
    });
    if (!r.ok) throw new Error(`${url} HTTP ${r.status}`);
    return await r.json();
  }

  function isQorA(e) {
    const key = String(e.key || '').toLowerCase();
    const code = String(e.code || '');
    const isQ = key === 'q' || code === 'KeyQ';
    const isA = key === 'a' || code === 'KeyA';
    return { isQ, isA, ok: isQ || isA };
  }

  function blurEditorIfNeeded() {
    try {
      const el = document.activeElement;
      const tag = String(el?.tagName || '').toUpperCase();
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || el?.isContentEditable) {
        el.blur?.();
      }
    } catch (_) {}
  }

  async function currentZoomState() {
    try {
      const z = await getJson('/api/zoom/state');
      return {
        idx: Number(z.zoom_sample_idx || 0),
        count: Number(z.zoom_sample_count || samples()) || samples()
      };
    } catch (_) {
      return { idx: 0, count: samples() };
    }
  }

  async function start(isQ, ev) {
    if (active) return;

    blurEditorIfNeeded();

    const st = await currentZoomState();
    const direction = isQ ? 1 : -1;
    const cmd = isQ ? teleCmd() : wideCmd();

    active = {
      key: isQ ? 'Q' : 'A',
      direction,
      cmd,
      startIdx: Math.max(0, Math.min(Math.max(1, st.count) - 1, st.idx)),
      count: Math.max(2, st.count),
      startTs: performance.now()
    };

    const payload = {
      action: 'start',
      cmd,
      hold_ms: fullSweepMs(),
      source: `hard_override_${active.key}`
    };

    const res = await postJson('/api/zoom/jog', payload);

    console.log('[qa-hard-jog] START', { payload, res, active });
    try { window.ptzLog?.('QA HARD JOG START', { payload, res, active }); } catch (_) {}

    if ($('ptzTuneStatus')) {
      $('ptzTuneStatus').textContent =
        `Q/A backend jog START ${active.key} cmd=${cmd} sample=${active.startIdx}`;
    }
  }

  async function stop(reason = 'keyup') {
    if (!active) return;

    const h = active;
    active = null;

    let stopRes = null;
    try {
      stopRes = await postJson('/api/zoom/jog', {
        action: 'stop',
        cmd: 0,
        source: `hard_override_${h.key}_stop`
      });
    } catch (e) {
      console.warn('[qa-hard-jog] STOP failed', e);
      try { await postJson('/api/zoom/jog', { action: 'stop', cmd: 0 }); } catch (_) {}
    }

    const heldMs = Math.max(0, performance.now() - h.startTs);
    const rawDelta = heldMs / Math.max(1, stepMs());
    const delta = Math.max(0, Math.round(rawDelta));

    const maxIdx = Math.max(0, h.count - 1);
    const target = Math.max(0, Math.min(maxIdx, h.startIdx + h.direction * delta));
    const ratio = maxIdx > 0 ? target / maxIdx : 0;

    let stateRes = null;
    try {
      stateRes = await postJson('/api/zoom/state', {
        zoom_sample_idx: target,
        zoom_ratio: ratio,
        zoom_confidence: 0.75,
        zoom_source: 'qa_hard_backend_jog'
      });
    } catch (e) {
      console.warn('[qa-hard-jog] state update failed', e);
    }

    try {
      await window.initPtzSpeedTuneSamples?.();
    } catch (_) {
      try { window.updatePtzTuneSampleButtonUi?.(); } catch (_) {}
    }

    console.log('[qa-hard-jog] STOP', {
      reason,
      key: h.key,
      start_idx: h.startIdx,
      target_idx: target,
      held_ms: Math.round(heldMs),
      raw_delta: Number(rawDelta.toFixed(2)),
      delta,
      step_ms: Number(stepMs().toFixed(1)),
      stopRes,
      stateRes
    });

    try {
      window.ptzLog?.('QA HARD JOG STOP', {
        reason,
        key: h.key,
        start_idx: h.startIdx,
        target_idx: target,
        held_ms: Math.round(heldMs),
        delta,
        stopRes,
        stateRes
      });
    } catch (_) {}

    if ($('ptzTuneStatus')) {
      $('ptzTuneStatus').textContent =
        `Q/A backend jog: ${h.startIdx}→${target}, held=${Math.round(heldMs)}ms step=${stepMs().toFixed(1)}ms`;
    }
  }

  window.addEventListener('keydown', (e) => {
    const k = isQorA(e);
    if (!k.ok) return;
    if (!isSweep()) return;

    e.preventDefault();
    e.stopPropagation();
    e.stopImmediatePropagation();

    if (e.repeat) return;

    start(k.isQ, e).catch(err => {
      console.error('[qa-hard-jog] START error', err);
      active = null;
      postJson('/api/zoom/jog', { action: 'stop', cmd: 0 }).catch(() => {});
    });
  }, true);

  window.addEventListener('keyup', (e) => {
    const k = isQorA(e);
    if (!k.ok) return;
    if (!isSweep()) return;

    e.preventDefault();
    e.stopPropagation();
    e.stopImmediatePropagation();

    stop('keyup').catch(err => {
      console.error('[qa-hard-jog] STOP error', err);
      active = null;
      postJson('/api/zoom/jog', { action: 'stop', cmd: 0 }).catch(() => {});
    });
  }, true);

  window.addEventListener('blur', () => {
    if (active) stop('window_blur').catch(() => postJson('/api/zoom/jog', { action: 'stop', cmd: 0 }).catch(() => {}));
  });

  document.addEventListener('visibilitychange', () => {
    if (document.hidden && active) stop('visibility_hidden').catch(() => postJson('/api/zoom/jog', { action: 'stop', cmd: 0 }).catch(() => {}));
  });

  window.qaHardJogStop = () => stop('manual_console');
  console.log('[qa-hard-jog] installed');
})();
// QA_SWEEP_BACKEND_JOG_HARD_OVERRIDE_END
</script>
'''

if "QA_SWEEP_BACKEND_JOG_HARD_OVERRIDE_START" in s:
    print("SKIP: hard override already installed")
else:
    if "</body>" in s:
        s = s.replace("</body>", block + "\n</body>", 1)
    elif "</html>" in s:
        s = s.replace("</html>", block + "\n</html>", 1)
    else:
        s += "\n" + block + "\n"

    p.write_text(s, encoding="utf-8")
    print("OK: installed hard Q/A backend jog override")
