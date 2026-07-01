from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_force_auto_sweep_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

changed = False

# Ensure payloadForAnchor includes forced sweep fields.
old_payload = """      wide_hold_ms: num('zoomWideHoldMs', 1500),
      calibration_direction: $('zoomCalibDirection')?.value || 'wide_to_tele',
"""

new_payload = """      wide_hold_ms: num('zoomWideHoldMs', 1500),

      // RUN FULL CALIB is AUTO mode: always use sweep-time sample movement.
      zoom_move_mode: 'sweep_time_steps',
      full_sweep_ms: num('zoomFullSweepMs', num('zoomWideHoldMs', 1500)),

      calibration_direction: $('zoomCalibDirection')?.value || 'wide_to_tele',
"""

if "RUN FULL CALIB is AUTO mode: always use sweep-time sample movement" not in s:
    if old_payload not in s:
        print("WARN: exact payload insert point not found; trying alternate patch")

        alt_payload = """      zoom_move_mode: $('zoomMoveMode')?.value || 'legacy_impulse',
      full_sweep_ms: num('zoomFullSweepMs', num('zoomWideHoldMs', 1500)),
"""
        forced_payload = """      // RUN FULL CALIB is AUTO mode: always use sweep-time sample movement.
      zoom_move_mode: 'sweep_time_steps',
      full_sweep_ms: num('zoomFullSweepMs', num('zoomWideHoldMs', 1500)),
"""
        if alt_payload in s:
            s = s.replace(alt_payload, forced_payload, 1)
            print("OK: replaced selected movement mode with forced AUTO SWEEP in RUN FULL CALIB payload")
            changed = True
        else:
            raise SystemExit("ERROR: cannot find payload patch point")
    else:
        s = s.replace(old_payload, new_payload, 1)
        print("OK: inserted forced AUTO SWEEP fields into RUN FULL CALIB payload")
        changed = True
else:
    print("SKIP: forced AUTO SWEEP payload already present")

# Force selector to sweep at start of runFullCalibration.
old_start = """      const anchors = enabledAnchors();

      if (!anchors.length) {
"""

new_start = """      const anchors = enabledAnchors();

      // RUN FULL CALIB = automatic sweep-time mode.
      if ($('zoomMoveMode')) $('zoomMoveMode').value = 'sweep_time_steps';

      const samplesAuto = num('zoomSamples', 10);
      const fullSweepAuto = num('zoomFullSweepMs', num('zoomWideHoldMs', 1500));
      const stepMsAuto = fullSweepAuto / Math.max(1, samplesAuto - 1);

      setZoomStatus(`FULL CALIB AUTO SWEEP: samples=${samplesAuto} full=${fullSweepAuto}ms step=${stepMsAuto.toFixed(1)}ms`);

      if (!anchors.length) {
"""

if "FULL CALIB AUTO SWEEP:" not in s:
    if old_start not in s:
        raise SystemExit("ERROR: runFullCalibration start insert point not found")
    s = s.replace(old_start, new_start, 1)
    print("OK: RUN FULL CALIB now forces UI selector to SWEEP")
    changed = True
else:
    print("SKIP: runFullCalibration already forces AUTO SWEEP")

# Make saveSettingsForAnchor log/verify sweep payload.
old_save = """  async function saveSettingsForAnchor(anchor) {
    const payload = payloadForAnchor(anchor);
    await postJson('/api/zoom_calibration/settings', payload);
    return payload;
  }
"""

new_save = """  async function saveSettingsForAnchor(anchor) {
    const payload = payloadForAnchor(anchor);
    payload.zoom_move_mode = 'sweep_time_steps';
    payload.full_sweep_ms = num('zoomFullSweepMs', num('zoomWideHoldMs', 1500));

    const res = await postJson('/api/zoom_calibration/settings', payload);

    const savedMode = res?.zoom_move_mode || payload.zoom_move_mode;
    if (savedMode !== 'sweep_time_steps') {
      throw new Error(`AUTO SWEEP save failed: backend mode=${savedMode}`);
    }

    console.log('[zoom-full-calib] AUTO SWEEP settings saved', {
      mode: payload.zoom_move_mode,
      samples: payload.samples,
      full_sweep_ms: payload.full_sweep_ms,
      step_ms: payload.full_sweep_ms / Math.max(1, payload.samples - 1),
      backend: res
    });

    return payload;
  }
"""

if "AUTO SWEEP settings saved" not in s:
    if old_save not in s:
        print("WARN: saveSettingsForAnchor block not found; skipped verify patch")
    else:
        s = s.replace(old_save, new_save, 1)
        print("OK: saveSettingsForAnchor now verifies AUTO SWEEP")
        changed = True
else:
    print("SKIP: saveSettingsForAnchor already verifies AUTO SWEEP")

# Improve wait status display to show backend progress mode/hold if available.
old_status = """      const msg = st.message || st.last_message || st.status || '';

      setZoomStatus(`RUN ${label}: ${inProgress ? 'running' : 'done'} idx=${idx}/${total} tags=${tags} ${msg}`);
"""

new_status = """      const msg = st.message || st.last_message || st.status || '';
      const pr = st.progress || {};
      const mode = pr.mode || st.zoom_move_mode || '';
      const hold = pr.move_hold_ms ?? pr.hold_ms ?? '';
      const delta = pr.move_delta ?? '';
      const step = pr.step_ms ?? '';

      const moveInfo = mode
        ? ` mode=${mode} delta=${delta} hold=${hold}ms step=${step}`
        : '';

      setZoomStatus(`RUN ${label}: ${inProgress ? 'running' : 'done'} idx=${idx}/${total} tags=${tags}${moveInfo} ${msg}`);
"""

if "const moveInfo = mode" not in s:
    if old_status not in s:
        print("WARN: waitCalibrationDone status block not found; skipped status patch")
    else:
        s = s.replace(old_status, new_status, 1)
        print("OK: RUN FULL CALIB status now shows mode/delta/hold/step")
        changed = True
else:
    print("SKIP: RUN FULL CALIB status already shows moveInfo")

if changed:
    p.write_text(s, encoding="utf-8")
    print("DONE")
else:
    print("No changes")
