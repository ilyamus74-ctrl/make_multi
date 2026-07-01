from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_run_full_calib_auto_movement_mode_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

changed = False

# 1. Ensure RUN FULL CALIB payload passes new movement fields.
old_payload = """      wide_hold_ms: num('zoomWideHoldMs', 1500),
      calibration_direction: $('zoomCalibDirection')?.value || 'wide_to_tele',
"""

new_payload = """      wide_hold_ms: num('zoomWideHoldMs', 1500),
      zoom_move_mode: $('zoomMoveMode')?.value || 'legacy_impulse',
      full_sweep_ms: num('zoomFullSweepMs', num('zoomWideHoldMs', 1500)),
      calibration_direction: $('zoomCalibDirection')?.value || 'wide_to_tele',
"""

if "zoom_move_mode: $('zoomMoveMode')?.value" not in s:
    if old_payload not in s:
        raise SystemExit("ERROR: payload insert point not found")
    s = s.replace(old_payload, new_payload, 1)
    print("OK: RUN FULL CALIB payload now includes zoom_move_mode/full_sweep_ms")
    changed = True
else:
    print("SKIP: payload already includes zoom_move_mode")

# 2. Before clear/run, force-save current movement mode and verify backend accepted it.
old_run = """      const names = anchors.map(a => `${a.label}:id=${a.tag_id}@${(a.distance_mm / 1000).toFixed(2)}m`).join(', ');
      setZoomStatus(`FULL CALIB: anchors=${names}`);

      await clearOldProfilesOnce();
"""

new_run = """      const moveMode = $('zoomMoveMode')?.value || 'legacy_impulse';
      const samples = num('zoomSamples', 10);
      const fullSweepMs = num('zoomFullSweepMs', num('zoomWideHoldMs', 1500));
      const stepMs = fullSweepMs / Math.max(1, samples - 1);

      const names = anchors.map(a => `${a.label}:id=${a.tag_id}@${(a.distance_mm / 1000).toFixed(2)}m`).join(', ');

      setZoomStatus(
        moveMode === 'sweep_time_steps'
          ? `FULL CALIB AUTO: saving mode=sweep time steps samples=${samples} full=${fullSweepMs}ms step=${stepMs.toFixed(1)}ms`
          : `FULL CALIB AUTO: saving mode=legacy impulse samples=${samples}`
      );

      // Force-save current UI mode before clearing/running.
      await saveSettingsForAnchor(anchors[0]);

      const savedSettings = await getJson('/api/zoom_calibration/settings').catch(() => null);
      if (savedSettings) {
        const savedMode = savedSettings.zoom_move_mode === 'sweep_time_steps' ? 'sweep_time_steps' : 'legacy_impulse';
        if (savedMode !== moveMode) {
          throw new Error(`movement mode was not saved: ui=${moveMode} backend=${savedMode}`);
        }
      }

      setZoomStatus(
        moveMode === 'sweep_time_steps'
          ? `FULL CALIB: mode=sweep time steps step=${stepMs.toFixed(1)}ms anchors=${names}`
          : `FULL CALIB: mode=legacy impulse anchors=${names}`
      );

      await clearOldProfilesOnce();
"""

if "FULL CALIB AUTO: saving mode=" not in s:
    if old_run not in s:
        raise SystemExit("ERROR: runFullCalibration insert point not found")
    s = s.replace(old_run, new_run, 1)
    print("OK: RUN FULL CALIB now force-saves and verifies movement mode before run")
    changed = True
else:
    print("SKIP: RUN FULL CALIB auto-save already installed")

# 3. Improve confirm dialog so operator sees which movement mode will be used.
old_confirm = """      const anchors = enabledAnchors();
      const msg = [
        `Run full calibration for ${anchors.length} anchors?`,
        '',
        ...anchors.map(a => `${a.label}: id=${a.tag_id}, distance=${(a.distance_mm / 1000).toFixed(2)}m`),
        '',
        'This will archive old zoom calibration profiles, build master profile, and generate PTZ speed profile.'
      ].join('\\n');
"""

new_confirm = """      const anchors = enabledAnchors();
      const moveMode = $('zoomMoveMode')?.value || 'legacy_impulse';
      const samples = num('zoomSamples', 10);
      const fullSweepMs = num('zoomFullSweepMs', num('zoomWideHoldMs', 1500));
      const stepMs = fullSweepMs / Math.max(1, samples - 1);

      const movementLine = moveMode === 'sweep_time_steps'
        ? `Movement: SWEEP TIME STEPS, samples=${samples}, full=${fullSweepMs}ms, step=${stepMs.toFixed(1)}ms`
        : `Movement: LEGACY IMPULSE, samples=${samples}, impulse=${num('zoomImpulseMs', 170)}ms`;

      const msg = [
        `Run full automatic calibration for ${anchors.length} anchors?`,
        '',
        movementLine,
        '',
        ...anchors.map(a => `${a.label}: id=${a.tag_id}, distance=${(a.distance_mm / 1000).toFixed(2)}m`),
        '',
        'This will archive old zoom calibration profiles, run anchors, build master profile, and generate PTZ speed profile.'
      ].join('\\n');
"""

if "Run full automatic calibration" not in s:
    if old_confirm not in s:
        print("WARN: confirm dialog block not found; skipped confirm text patch")
    else:
        s = s.replace(old_confirm, new_confirm, 1)
        print("OK: confirm dialog now shows movement mode")
        changed = True
else:
    print("SKIP: confirm dialog already patched")

if not changed:
    print("No changes needed")
else:
    p.write_text(s, encoding="utf-8")
    print("DONE")
