from pathlib import Path
import time

ROOT = Path('/root/new_yolo8') if Path('/root/new_yolo8').exists() else Path.cwd()
p = ROOT / 'ptz_contract_audit.py'

s = p.read_text(encoding='utf-8', errors='ignore')

marker = 'PTZ_AUDIT_UI_DETECTION_JS_INLINE_V3'

if marker in s:
    print('already patched:', marker)
    raise SystemExit(0)

bak = p.with_suffix(p.suffix + f'.bak_ui_js_inline_v3_{int(time.time())}')
bak.write_text(s, encoding='utf-8')

layer_pos = s.find('LAYER 1 — BROWSER UI')
if layer_pos < 0:
    layer_pos = s.find('LAYER 1')
if layer_pos < 0:
    raise SystemExit('cannot find Layer 1 section in ptz_contract_audit.py')

# Find containing function start before Layer 1 and next function after it.
func_start = s.rfind('\ndef ', 0, layer_pos)
if func_start < 0:
    func_start = s.rfind('def ', 0, layer_pos)
if func_start < 0:
    raise SystemExit('cannot find function containing Layer 1 section')

next_def = s.find('\ndef ', layer_pos)
if next_def < 0:
    raise SystemExit('cannot find end of Layer 1 function')

insert = r'''

    # PTZ_AUDIT_UI_DETECTION_JS_INLINE_V3
    section("LAYER 1B — UI DETECTION CONTROLS JS CONTRACT")
    from pathlib import Path as _PtzAuditPath
    try:
        web_text = _PtzAuditPath("web/index.html").read_text(encoding="utf-8", errors="ignore")
    except Exception as e:
        web_text = ""
        add("UI JS contract web/index.html readable", False, str(e))

    required_js_markers = [
        "PTZ_CLEAN_DETECTION_UI_HYDRATE_F5_START",
        "PTZ_CLEAN_DETECTION_ACTIVE_PRESET_PERSIST_START",
        "hydrateDetectionControlsFromSettings",
        "scheduleDetectionUiHydrate",
        "persistActiveDetectionControlsToPreset",
        "schedulePersistActiveDetectionControlsToPreset",
    ]

    for marker_name in required_js_markers:
        cnt = web_text.count(marker_name)
        print(f"{marker_name} = {cnt}")
        add(f"UI detection JS marker {marker_name}", cnt > 0, f"count={cnt}")

    # UI must not be able to destroy presets by writing an empty objectPresetsCustom
    # from old saveSettings paths. The clean save guard must fall back to OBJECT_PRESETS
    # when the server-side objectPresetsCustom is temporarily empty.
    has_presets_fallback = (
        "objectPresetsCustom" in web_text
        and "OBJECT_PRESETS" in web_text
        and "Object.keys(before.objectPresetsCustom).length" in web_text
    )
    print("objectPresetsCustom fallback guard =", has_presets_fallback)
    add(
        "UI saveSettings preserves objectPresetsCustom fallback",
        has_presets_fallback,
        "requires OBJECT_PRESETS fallback when before.objectPresetsCustom is empty"
    )

    # UI detection controls are not just runtime controls. They must persist into
    # objectPresetsCustom[activeObjectPreset] so F5/re-select restores operator choices.
    active_preset_write_markers = [
        "max_detections",
        "detect_every_n_frames",
        "detection_mode",
        "activeObjectPreset",
        "objectPresetsCustom",
    ]
    missing_persist_parts = [m for m in active_preset_write_markers if m not in web_text]
    print("active preset detection persist missing parts =", missing_persist_parts)
    add(
        "UI detection controls persist to active object preset",
        not missing_persist_parts,
        f"missing={missing_persist_parts}"
    )
'''

s = s[:next_def] + insert + s[next_def:]
p.write_text(s, encoding='utf-8')

print('OK patched ptz_contract_audit.py with inline UI JS audit v3')
print('Backup:', bak)
print('Inserted inside function starting at byte:', func_start)