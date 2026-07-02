# PTZ Object Tracking — Master Contract

## Назначение файла

Этот файл является главным контрактом архитектуры PTZ/Object Tracking проекта.

Перед любым изменением кода нужно:

1. Определить, какой слой меняется.
2. Проверить контракт слоя.
3. Проверить, не нарушается ли соседний слой.
4. Сделать audit до изменения.
5. Внести изменение.
6. Сделать audit после изменения.
7. Если контракт изменился — обновить этот файл.

---

# Главная схема

```text
┌─────────────────────────────────────────────┐
│ 1. Browser UI                               │
│ web/index.html                              │
│                                             │
│ Тонкий клиент: read / post / verify         │
└─────────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────┐
│ 2. Settings / Presets                       │
│ /api/settings + ui_settings.json            │
│                                             │
│ Единый источник состояния и пресетов        │
└─────────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────┐
│ 5. Orchestrator / Supervisor                │
│ object_tracking_daemon.py                   │
│                                             │
│ ptzArmed true/false → start/stop runtime    │
└─────────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────┐
│ 6. Runtime Strategy Engine                  │
│ apply_ptz_object_preset.py --watch          │
│                                             │
│ object preset + search preset + tracking    │
└─────────────────────────────────────────────┘
          │                         │
          ▼                         ▼
┌───────────────────────┐   ┌───────────────────────┐
│ 3. Detector backend   │   │ 4. PTZ autopilot       │
│ mjpeg_gst_http :8080  │   │ ptz_autopilot :8090    │
│                       │   │                       │
│ detection/tracker     │   │ movement controller    │
└───────────────────────┘   └───────────────────────┘
Layer 1 — Browser UI
Файл
web/index.html
Роль

Browser UI — только панель управления. Это не главный источник состояния.

UI читает backend, отображает состояние, отправляет изменения и проверяет результат обратным чтением.

Можно
Показывать текущий object preset.
Показывать текущий search preset.
Показывать Model / Limit / Detect FPS / Area.
Отправлять изменения в backend.
После отправки читать backend и подтверждать состояние.
Показывать PTZ STOPPED / ARMED / ACTIVE.
Нельзя
Самостоятельно включать PTZ при выборе object preset.
Самостоятельно выбирать дефолтный activeObjectPreset, если backend уже имеет состояние.
Хранить object presets как главный источник истины.
Использовать localStorage как главный источник истины.
Запускать runtime tracking.
Напрямую запускать apply_ptz_object_preset.py.
Писать ptzArmed=true где-либо, кроме кнопки START PTZ.
Контракт
OBJECT PRESET click:
  POST detection/framing config
  POST activeObjectPreset
  POST ptzArmed=false
  READ /api/settings
  VERIFY activeObjectPreset

START PTZ:
  POST ptzArmed=true
  READ /api/settings
  VERIFY ptzArmed=true

STOP PTZ:
  POST ptzArmed=false
  POST /api/autopilot/stop
  READ /api/settings
  VERIFY ptzArmed=false
Обязательные UI-маркеры
PTZ_CLEAN_LAYER_CONTROLLER_START = 1
PTZ_CLEAN_SAVESETTINGS_GUARD_START = 1
Запрещённые UI-маркеры
SINGLE_PRESET_AUTO_ARM = 0
OBJECT_PRESET_MANAGER_START = 0
OBJECT_PRESET_BUTTONS_ACTIVE_FIX_START = 0
OBJECT_PRESET_AUTO_LEARN_OPERATOR_CONTROLS_START = 0
OPERATOR_CONTROLS_HARD_PERSIST_APPLY_START = 0
OBJECT_PRESET_NO_AUTO_ARM_DURABLE_SAVE_START = 0
PTZ_START_STOP_ACTIVE_STATE_FIX_START = 0
SEARCH_PRESET_SELECTOR_START = 0
Layer 2 — Settings / Presets
Runtime API
/api/settings
Persistent file
ui_settings.json
Helper processes
hydrate_runtime_settings.py
settings_persist_daemon.py
Роль

Settings layer — главный источник состояния и пресетов.

ui_settings.json хранит persistent-состояние.
/api/settings — runtime mirror.
hydrate_runtime_settings.py поднимает file → runtime на старте.
settings_persist_daemon.py сохраняет runtime → file во время работы.

Хранит
activeObjectPreset
objectPresetsCustom
activeSearchPreset
searchPresetsCustom
ptzArmed
operatorModel
operatorDetectionLimit
operatorDetectEvery
operatorDetectionAreaMode
objectPresetTrackingMode
objectPresetLossBehavior
lastAppliedObjectPreset
lastAppliedSearchPreset
Правило безопасности

В ui_settings.json не сохраняем автозапуск PTZ после рестарта.

file ptzArmed = false
file controlMode = manual

Runtime /api/settings может иметь:

ptzArmed = true
controlMode = ptz

только во время активной работы.

Object preset содержит
label
classes
model
operatorModel
detection_mode
max_detections
max_raw_candidates
detect_every_n_frames
tracking_mode
loss_behavior
ptz.target_x
ptz.target_y
ptz.auto_zoom_enable
ptz.auto_zoom_target_h
ptz.auto_zoom_deadzone
ptz.auto_zoom_cmd
ptz.auto_zoom_sign
ptz.auto_zoom_period_ms
Object preset не содержит
kp
ki
kd
deadzone
max_pan
max_tilt
max_accel
min_pan
min_tilt
hz
ptz_curve
ptz_lead_ms
manual_mode
j_pulse_ms

Эти параметры относятся к PTZ SPEED TUNE, а не к object preset.

Layer 3 — Detector backend
Файл
mjpeg_gst_http.cpp
Port
8080
Роль

Detector backend отвечает за видео, модель, детекцию, tracker state и zoom profile state.

Основные API
/api/detector/config
/api/detection/limits
/api/detection/throttle
/api/detection/roi_config
/api/tracker/state
/api/detections
/api/zoom/state
/api/zoom/go_to_sample
/api/zoom_calibration/settings
Можно
Менять RKNN model.
Менять selected classes.
Менять detection limit.
Менять detect every N frames.
Менять ROI/tiled/hybrid mode.
Давать tracker state.
Управлять zoom через profile frames.
Нельзя
Решать, когда включать PTZ.
Знать бизнес-логику object preset.
Знать search preset как сценарий.
Писать ptzArmed.
Стартовать autopilot.
Zoom contract

Search/runtime zoom должен идти через:

/api/zoom/go_to_sample

Нельзя использовать raw zoom jog для search strategy, если нужно синхронизировать PTZ SPEED TUNE с zoom frame.

Layer 4 — PTZ autopilot backend
Файл
ptz_autopilot.cpp
Port
8090
Роль

PTZ autopilot — низкоуровневый контроллер движения камеры.

Основные API
/api/autopilot/config
## Framing-only fields

text
target_x
target_y
auto_zoom_enable
auto_zoom_target_h
auto_zoom_deadzone
auto_zoom_cmd
auto_zoom_sign
auto_zoom_period_ms

Framing-only config must not change PTZ speed profile source.

Expected:

speed_profile_source must remain user/exact/interpolated/clamped_left/clamped_right
speed_profile_source must not become runtime_override

Speed tune fields

kp
ki
kd
deadzone
max_pan
max_tilt
max_accel
min_pan
min_tilt
hz

Only these fields are allowed to create runtime speed override.
Rule

Object preset may send framing-only config.
Object preset must not create PTZ speed override.

/api/autopilot/start
/api/autopilot/stop
/api/autopilot/state
/api/autopilot/speed_profile/apply_nearest
/api/control/manual_j_pulse
/api/control/stop
Можно
Стартовать/останавливать autopilot по явной команде runtime/UI.
Принимать framing config.
Принимать PTZ SPEED TUNE.
Вычислять pan/tilt команды.
Исполнять manual pulse.
Отдавать current PTZ state.
Нельзя
Читать ui_settings.json напрямую.
Знать object preset.
Знать search preset.
Самостоятельно решать, кого искать.
Самостоятельно запускать tracking.
Самостоятельно включаться от выбора object preset.
Future extension

Анализ траектории объекта добавлять как отдельный слой между Runtime Strategy и PTZ Autopilot:

Runtime Strategy
  ↓
Trajectory Analyzer
  ↓
PTZ Autopilot



Layer 5 — Orchestrator / Supervisor
Файл
object_tracking_daemon.py
Роль

Daemon управляет жизненным циклом runtime tracking.

Он не трекает объект и не крутит камеру. Он только следит за состоянием и запускает/останавливает runtime.

Можно
Читать /api/settings.
Смотреть ptzArmed.
Смотреть activeObjectPreset.
Смотреть activeSearchPreset.
Запускать apply_ptz_object_preset.py --watch.
Останавливать apply_ptz_object_preset.py --watch.
Не допускать двух watch-процессов.
Перезапускать runtime при смене object/search preset.
Нельзя
Самостоятельно применять detection config.
Самостоятельно включать autopilot.
Самостоятельно искать объект.
Самостоятельно менять presets.
Самостоятельно писать ptzArmed=true.
Контракт
ptzArmed=false:
  watch_count = 0

ptzArmed=true:
  watch_count = 1

activeObjectPreset changed:
  restart watch

activeSearchPreset changed:
  restart watch
Layer 6 — Runtime Strategy Engine
Файл
apply_ptz_object_preset.py
Runtime mode
apply_ptz_object_preset.py --watch
Роль

Runtime Strategy Engine исполняет текущий object/search preset.

Можно
Читать active object/search preset.
Применять detector config.
Применять ROI/limits/throttle.
Применять PTZ framing.
Пытаться захватить объект.
При найденной цели стартовать autopilot.
При потере цели выполнять search preset.
Использовать zoom frame movement через /api/zoom/go_to_sample.
Использовать manual_j_pulse для search movement.
Нельзя
Хранить persistent settings.
Затирать objectPresetsCustom.
Менять PTZ SPEED TUNE из object preset.
Стартовать без ptzArmed=true.
Делать raw zoom jog вместо zoom frame profile.
Search Preset Contract

Search preset отвечает только за поведение после потери цели.

Может содержать
name
label
loop
lost_grace_sec
steps
zoom_delta_frames
pan_direction
tilt_direction
hold_ms
pause_sec
acquire_attempts
Не должен менять
model
classes
detection limit
detect every
object preset
PTZ SPEED TUNE
Development Process Contract

Перед любым изменением:

1. Определить слой.
2. Проверить контракт слоя.
3. Сделать audit.
4. Внести patch.
5. Сделать audit после patch.
6. Если изменился контракт — обновить PTZ_MASTER_CONTRACT.md.
Запрещено
- Делать overlay patch без аудита.
- Добавлять второй handler на тот же UI control без удаления старого.
- Менять несколько слоёв без явного объяснения.
- Сохранять persistent ptzArmed=true.
- Применять object preset и одновременно стартовать PTZ.
Разрешено менять несколько слоёв только если
- явно указано, какие слои меняются
- описано, почему нельзя ограничиться одним слоем
- audit до/после подтверждает отсутствие конфликта
Required Audits
UI audit

Проверяет:

PTZ_CLEAN_LAYER_CONTROLLER_START = 1
PTZ_CLEAN_SAVESETTINGS_GUARD_START = 1
SINGLE_PRESET_AUTO_ARM = 0
старые object preset handlers = 0
Settings audit

Проверяет:

API custom keys не пустые
FILE custom keys не пустые
API activeObjectPreset == FILE activeObjectPreset
API activeSearchPreset == FILE activeSearchPreset
FILE ptzArmed = false
FILE controlMode = manual
Detector audit

Проверяет:

/api/detection/limits == active preset max_detections
/api/detection/throttle == active preset detect_every_n_frames
/api/detection/roi_config == active preset detection_mode
/api/detector/config selected_classes == active preset classes
PTZ audit

Проверяет:

autopilot.enabled=false после STOP/restart
object preset не стартует autopilot
speed_profile_source=user
Daemon audit

Проверяет:

один object_tracking_daemon.py
ptzArmed=false → watch_count=0
ptzArmed=true → watch_count=1
ptzArmed=false → watch_count=0
Current known stable point

После clean-controller v4:

Browser UI:
  object preset не включает PTZ сам
  activeObjectPreset не прыгает
  LIMIT/FPS сохраняются
  ptzArmed остаётся false до START PTZ

После runtime test:

Daemon:
  ptzArmed=false → watch_count=0
  ptzArmed=true → watch_count=1
  ptzArmed=false → watch_count=0

После restore/hydrate/persist:

Settings:
  API custom keys = ['car_single', 'person_single']
  FILE custom keys = ['car_single', 'person_single']
  settings_persist_daemon.py running
  object_tracking_daemon.py running



##########################
Да. ptz_contract_audit.py теперь нельзя удалять. Это часть проекта, как тесты.

Роль файлов:

PTZ_MASTER_CONTRACT.md      — архитектурный закон проекта
ptz_contract_audit.py       — проверка, что код не нарушил закон
ptz_contract_audit_last.json — временный отчёт последнего аудита, можно удалять
старые patch_*.py           — можно удалять после успешного внедрения

После каждого вмешательства в слои:

1. обновили код
2. запустили ptz_contract_audit.py
3. увидели PASS/FAIL
4. если контракт поменялся — обновили PTZ_MASTER_CONTRACT.md
5. только потом считаем изменение закрытым

# Mandatory Project Files

These files are part of the project contract and must not be deleted:


Generated audit output can be deleted:

ptz_contract_audit_last.json

Old one-time patch files can be deleted after the target code is patched and audit passes.

Rule:

Any layer change must be followed by:
  1. ptz_contract_audit.py run
  2. review of FAIL items
  3. fix or explicit contract update

no delete files
ptz_contract_audit.py
hydrate_runtime_settings.py
settings_persist_daemon.py
object_tracking_daemon.py
apply_ptz_object_preset.py
##########################