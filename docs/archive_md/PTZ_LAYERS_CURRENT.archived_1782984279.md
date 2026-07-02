# PTZ Object Tracking Layers

## Цель

Разделить ответственность между слоями, чтобы OBJECT PRESET, SEARCH PRESET и START PTZ не конфликтовали.

---

## 1. Browser UI: `web/index.html`

Отвечает только за ввод пользователя и отображение состояния.

### Разрешено

- Выбор object preset.
- Выбор search preset.
- Изменение Model / Limit / Detect FPS / Area.
- Кнопка `SAVE DETECTION TO PRESET`.
- Кнопка `START PTZ`.
- Кнопка `STOP PTZ`.
- Отображение ACTIVE / ARMED / STOPPED.

### Запрещено

- Object preset не должен сам включать PTZ.
- Object preset не должен писать `ptzArmed=true`.
- Object preset не должен вызывать `/api/autopilot/start`.

### Правило

`ptzArmed=true` пишет только кнопка `START PTZ`.

---

## 2. Settings layer: `/api/settings` / `ui_settings.json`

Главный persistent-state.

### Основные поля

```json
{
  "activeObjectPreset": "car_single",
  "objectPresetsCustom": {},
  "activeSearchPreset": "lost_step_wait",
  "ptzArmed": false,
  "operatorModel": "...rknn",
  "operatorDetectionLimit": 8,
  "operatorDetectEvery": 1,
  "operatorDetectionAreaMode": "full_frame"
}
Правило

objectPresetsCustom хранит detection/framing/search metadata, но не PTZ SPEED TUNE.

3. Detector backend: mjpeg_gst_http.cpp / port 8080

Отвечает за video/detection/tracker/zoom-profile state.

APIs
/api/detector/config
/api/detection/limits
/api/detection/throttle
/api/detection/roi_config
/api/tracker/state
/api/zoom/state
/api/zoom/go_to_sample
Правило

Search zoom должен идти только через /api/zoom/go_to_sample, чтобы zoom frame и PTZ SPEED TUNE были синхронны.

4. PTZ autopilot backend: ptz_autopilot.cpp / port 8090

Отвечает за расчёт команд pan/tilt.

APIs
/api/autopilot/config
/api/autopilot/start
/api/autopilot/stop
/api/autopilot/state
/api/autopilot/speed_profile/apply_nearest
/api/control/stop
/api/control/manual_j_pulse
Правило

Object preset может менять только framing:

target_x
target_y
auto_zoom_enable
auto_zoom_target_h
auto_zoom_deadzone
auto_zoom_cmd
auto_zoom_sign
auto_zoom_period_ms

Object preset не должен менять:

kp
kd
deadzone
max_pan
max_tilt
max_accel
min_pan
min_tilt
5. Daemon layer: object_tracking_daemon.py

Следит за settings.

Логика
ptzArmed=false:
  stop child watch

ptzArmed=true:
  read activeObjectPreset
  read activeSearchPreset
  start apply_ptz_object_preset.py --watch
Правило

Daemon не решает сам, что нужно включить PTZ. Он только исполняет ptzArmed.

6. Runtime tracking: apply_ptz_object_preset.py --watch

Исполняет активный object/search preset.

Логика
apply object preset
try acquire target
if target found:
  start PTZ tracking
if target lost:
  execute active search preset
Правило

Runtime может стартовать autopilot только когда запущен daemon-ом при ptzArmed=true.

Итоговая схема
Browser UI
  ↓ writes
/api/settings / ui_settings.json
  ↓ read by daemon
object_tracking_daemon.py
  ↓ starts/stops
apply_ptz_object_preset.py --watch
  ↓ config
mjpeg_gst_http.cpp :8080
  ↓ target state
ptz_autopilot.cpp :8090
  ↓ commands
PTZ camera

