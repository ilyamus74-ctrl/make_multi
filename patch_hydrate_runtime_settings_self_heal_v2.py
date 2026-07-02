from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
p = ROOT / 'hydrate_runtime_settings.py'

if not p.exists():
    raise SystemExit('hydrate_runtime_settings.py not found')

bak = p.with_suffix(p.suffix + f'.bak_self_heal_v2_{int(time.time())}')
bak.write_text(p.read_text(encoding='utf-8', errors='replace'), encoding='utf-8')

content = r'''#!/usr/bin/env python3
import json
import time
import urllib.request
from pathlib import Path

ROOT = Path('/root/new_yolo8')
SETTINGS_FILE = ROOT / 'ui_settings.json'
MJPEG_BASE = 'http://127.0.0.1:8080'
PTZ_BASE = 'http://127.0.0.1:8090'

DEFAULT_MODEL = '/root/new_yolo8/model_rknn/yolov8s_800x800_9out_fp32.rknn'

CANONICAL_PRESETS = {
    'person_single': {
        'label': 'ЧЕЛОВЕК', 'classes': [0], 'detection_mode': 'full_frame',
        'max_detections': 5, 'max_raw_candidates': 25, 'detect_every_n_frames': 1,
        'tracking_mode': 'single_auto', 'loss_behavior': 'continuous_wide_scan_x',
        'ptz': {'target_x': 0.5, 'target_y': 0.46, 'auto_zoom_enable': True,
                'auto_zoom_target_h': 0.68, 'auto_zoom_deadzone': 0.08,
                'auto_zoom_cmd': 10, 'auto_zoom_sign': 1, 'auto_zoom_period_ms': 350}
    },
    'people': {
        'label': 'ЛЮДИ', 'classes': [0], 'detection_mode': 'full_frame',
        'max_detections': 10, 'max_raw_candidates': 50, 'detect_every_n_frames': 1,
        'tracking_mode': 'multi_operator', 'loss_behavior': 'operator_select',
        'ptz': {'target_x': 0.5, 'target_y': 0.47, 'auto_zoom_enable': True,
                'auto_zoom_target_h': 0.58, 'auto_zoom_deadzone': 0.10,
                'auto_zoom_cmd': 8, 'auto_zoom_sign': 1, 'auto_zoom_period_ms': 450}
    },
    'car_single': {
        'label': 'МАШИНА', 'classes': [2, 3, 5, 7], 'detection_mode': 'full_frame',
        'max_detections': 8, 'max_raw_candidates': 40, 'detect_every_n_frames': 1,
        'tracking_mode': 'single_auto', 'loss_behavior': 'continuous_wide_scan_x',
        'ptz': {'target_x': 0.5, 'target_y': 0.5, 'auto_zoom_enable': True,
                'auto_zoom_target_h': 0.48, 'auto_zoom_deadzone': 0.10,
                'auto_zoom_cmd': 8, 'auto_zoom_sign': 1, 'auto_zoom_period_ms': 450}
    },
    'cars': {
        'label': 'МАШИНЫ', 'classes': [2, 3, 5, 7], 'detection_mode': 'tiled',
        'max_detections': 15, 'max_raw_candidates': 80, 'detect_every_n_frames': 1,
        'tracking_mode': 'multi_operator', 'loss_behavior': 'operator_select',
        'ptz': {'target_x': 0.5, 'target_y': 0.5, 'auto_zoom_enable': True,
                'auto_zoom_target_h': 0.42, 'auto_zoom_deadzone': 0.12,
                'auto_zoom_cmd': 8, 'auto_zoom_sign': 1, 'auto_zoom_period_ms': 500}
    },
    'airplane_single': {
        'label': 'САМОЛЁТ', 'classes': [4], 'detection_mode': 'hybrid',
        'max_detections': 8, 'max_raw_candidates': 60, 'detect_every_n_frames': 1,
        'tracking_mode': 'single_auto', 'loss_behavior': 'continuous_wide_scan_x',
        'ptz': {'target_x': 0.5, 'target_y': 0.45, 'auto_zoom_enable': True,
                'auto_zoom_target_h': 0.30, 'auto_zoom_deadzone': 0.08,
                'auto_zoom_cmd': 10, 'auto_zoom_sign': 1, 'auto_zoom_period_ms': 350}
    },
    'airplanes': {
        'label': 'САМОЛЁТЫ', 'classes': [4], 'detection_mode': 'hybrid',
        'max_detections': 15, 'max_raw_candidates': 100, 'detect_every_n_frames': 1,
        'tracking_mode': 'multi_operator', 'loss_behavior': 'operator_select',
        'ptz': {'target_x': 0.5, 'target_y': 0.45, 'auto_zoom_enable': True,
                'auto_zoom_target_h': 0.25, 'auto_zoom_deadzone': 0.10,
                'auto_zoom_cmd': 10, 'auto_zoom_sign': 1, 'auto_zoom_period_ms': 400}
    },
    'bird_single': {
        'label': 'ПТИЦА', 'classes': [14], 'detection_mode': 'hybrid',
        'max_detections': 10, 'max_raw_candidates': 80, 'detect_every_n_frames': 1,
        'tracking_mode': 'single_auto', 'loss_behavior': 'continuous_wide_scan_x',
        'ptz': {'target_x': 0.5, 'target_y': 0.45, 'auto_zoom_enable': True,
                'auto_zoom_target_h': 0.24, 'auto_zoom_deadzone': 0.07,
                'auto_zoom_cmd': 12, 'auto_zoom_sign': 1, 'auto_zoom_period_ms': 300}
    },
    'birds': {
        'label': 'ПТИЦЫ', 'classes': [14], 'detection_mode': 'hybrid',
        'max_detections': 20, 'max_raw_candidates': 120, 'detect_every_n_frames': 1,
        'tracking_mode': 'multi_operator', 'loss_behavior': 'operator_select',
        'ptz': {'target_x': 0.5, 'target_y': 0.45, 'auto_zoom_enable': True,
                'auto_zoom_target_h': 0.20, 'auto_zoom_deadzone': 0.08,
                'auto_zoom_cmd': 12, 'auto_zoom_sign': 1, 'auto_zoom_period_ms': 300}
    },
}


def non_empty_dict(value):
    return isinstance(value, dict) and len(value) > 0


def read_file_settings():
    if not SETTINGS_FILE.exists():
        return {}
    try:
        return json.loads(SETTINGS_FILE.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return {}


def write_file_settings(data):
    SETTINGS_FILE.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def get_json(url, timeout=3):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        raw = r.read().decode('utf-8', errors='replace')
    return json.loads(raw) if raw.strip() else {}


def post_json(url, body, timeout=3):
    data = json.dumps(body).encode('utf-8')
    req = urllib.request.Request(
        url,
        data=data,
        headers={'Content-Type': 'application/json'},
        method='POST'
    )
    with urllib.request.urlopen(req, timeout=timeout) as r:
        raw = r.read().decode('utf-8', errors='replace')
    return json.loads(raw) if raw.strip() else {}


def wait_for_api_settings(timeout_sec=20):
    deadline = time.time() + timeout_sec
    last = None
    while time.time() < deadline:
        try:
            return get_json(f'{MJPEG_BASE}/api/settings', timeout=2)
        except Exception as e:
            last = e
            time.sleep(0.5)
    raise RuntimeError(f'/api/settings not ready: {last}')


def normalize_preset(name, preset, model):
    out = dict(preset or {})
    out['model'] = out.get('model') or model
    out['operatorModel'] = out.get('operatorModel') or model
    out.setdefault('label', name)
    out.setdefault('classes', [])
    out.setdefault('detection_mode', 'full_frame')
    out.setdefault('max_detections', 5)
    out.setdefault('max_raw_candidates', max(20, int(out.get('max_detections') or 5) * 4))
    out.setdefault('detect_every_n_frames', 1)
    out.setdefault('tracking_mode', 'single_auto')
    out.setdefault('loss_behavior', 'continuous_wide_scan_x')
    out.setdefault('ptz', {})
    return out


def build_canonical_custom(model):
    return {
        name: normalize_preset(name, preset, model)
        for name, preset in CANONICAL_PRESETS.items()
    }


def choose_custom(file_settings, runtime_settings, model):
    file_custom = file_settings.get('objectPresetsCustom')
    runtime_custom = runtime_settings.get('objectPresetsCustom')

    if non_empty_dict(file_custom):
        src = 'file'
        custom = file_custom
    elif non_empty_dict(runtime_custom):
        src = 'runtime'
        custom = runtime_custom
    else:
        src = 'canonical'
        custom = build_canonical_custom(model)

    return src, {
        name: normalize_preset(name, preset, model)
        for name, preset in custom.items()
    }


def apply_active_preset_to_runtime(merged, active_preset):
    model = active_preset.get('model') or active_preset.get('operatorModel') or DEFAULT_MODEL

    merged['operatorModel'] = model
    merged['detectorSelectedClasses'] = active_preset.get('classes') or []
    merged['operatorDetectionLimit'] = int(active_preset.get('max_detections') or 5)
    merged['operatorDetectEvery'] = int(active_preset.get('detect_every_n_frames') or 1)
    merged['operatorDetectionAreaMode'] = str(active_preset.get('detection_mode') or 'full_frame')
    merged['objectPresetTrackingMode'] = active_preset.get('tracking_mode') or 'single_auto'
    merged['objectPresetLossBehavior'] = active_preset.get('loss_behavior') or 'continuous_wide_scan_x'


def post_active_preset_to_backends(active_preset):
    model = active_preset.get('model') or active_preset.get('operatorModel') or DEFAULT_MODEL
    classes = active_preset.get('classes') or []

    post_json(f'{MJPEG_BASE}/api/detector/config', {
        'detect_enabled': True,
        'selected_classes': classes,
        'current_model': model,
    }, timeout=4)

    post_json(f'{MJPEG_BASE}/api/detection/limits', {
        'max_detections': int(active_preset.get('max_detections') or 5),
        'max_raw_candidates': int(active_preset.get('max_raw_candidates') or 20),
    }, timeout=4)

    post_json(f'{MJPEG_BASE}/api/detection/throttle', {
        'detect_every_n_frames': int(active_preset.get('detect_every_n_frames') or 1),
    }, timeout=4)

    post_json(f'{MJPEG_BASE}/api/detection/roi_config', {
        'detection_mode': str(active_preset.get('detection_mode') or 'full_frame'),
        'rois': [],
    }, timeout=4)

    ptz = active_preset.get('ptz') or {}
    if ptz:
        post_json(f'{PTZ_BASE}/api/autopilot/config', {
            'target_x': float(ptz.get('target_x', 0.5)),
            'target_y': float(ptz.get('target_y', 0.5)),
            'auto_zoom_enable': bool(ptz.get('auto_zoom_enable', True)),
            'auto_zoom_target_h': float(ptz.get('auto_zoom_target_h', 0.48)),
            'auto_zoom_deadzone': float(ptz.get('auto_zoom_deadzone', 0.10)),
            'auto_zoom_cmd': int(ptz.get('auto_zoom_cmd', 8)),
            'auto_zoom_sign': int(ptz.get('auto_zoom_sign', 1)),
            'auto_zoom_period_ms': int(ptz.get('auto_zoom_period_ms', 450)),
        }, timeout=4)


def main():
    file_settings = read_file_settings()
    runtime_settings = wait_for_api_settings()

    model = (
        file_settings.get('operatorModel')
        or runtime_settings.get('operatorModel')
        or DEFAULT_MODEL
    )

    custom_src, custom = choose_custom(file_settings, runtime_settings, model)

    merged = dict(file_settings)
    merged.update(runtime_settings)
    merged['objectPresetsCustom'] = custom

    active = (
        file_settings.get('activeObjectPreset')
        or runtime_settings.get('activeObjectPreset')
        or 'car_single'
    )
    if active not in custom:
        active = 'car_single' if 'car_single' in custom else next(iter(custom.keys()))

    merged['activeObjectPreset'] = active
    merged['activeSearchPreset'] = merged.get('activeSearchPreset') or 'lost_step_wait'
    merged['ptzArmed'] = False
    merged['controlMode'] = 'manual'

    active_preset = custom[active]
    apply_active_preset_to_runtime(merged, active_preset)

    merged['lastAppliedObjectPreset'] = {
        'name': active,
        'label': active_preset.get('label') or active,
        'tracking_mode': active_preset.get('tracking_mode') or 'single_auto',
        'loss_behavior': active_preset.get('loss_behavior') or 'continuous_wide_scan_x',
        'ts': int(time.time()),
        'source': 'hydrate_runtime_settings',
    }

    # Write persistent safe state before posting runtime state.
    # This prevents startup defaults from becoming the persistent source of truth.
    write_file_settings(merged)

    res = post_json(f'{MJPEG_BASE}/api/settings', merged, timeout=4)
    post_active_preset_to_backends(active_preset)

    print('OK hydrated runtime settings with self-heal')
    print('custom source =', custom_src)
    print('activeObjectPreset =', active)
    print('activeSearchPreset =', merged.get('activeSearchPreset'))
    print('ptzArmed =', merged.get('ptzArmed'))
    print('controlMode =', merged.get('controlMode'))
    print('custom presets =', sorted(custom.keys()))
    print('limit/every =', merged.get('operatorDetectionLimit'), merged.get('operatorDetectEvery'))
    print('area =', merged.get('operatorDetectionAreaMode'))
    print('api =', res)


if __name__ == '__main__':
    main()
'''

p.write_text(content, encoding='utf-8')
p.chmod(0o755)

print('OK replaced hydrate_runtime_settings.py with self-healing v2')
print('Backup:', bak)