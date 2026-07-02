# План доработки Pan-Tilt Tracker

## Цель

Сделать автономную систему слежения, где детекция, трекинг, выбор цели, PID и управление pan/tilt/zoom выполняются на сервере/железе, а браузер используется только как панель управления и мониторинг.

---

## 1. Архитектура управления

Текущая схема должна перейти к такой:

```text
camera / GStreamer
  → YOLO detector
  → ROI / tiled detection
  → stable object tracker
  → selected target
  → zoom-aware PID
  → ws_uart_bridge
  → UART
  → pan/tilt/zoom hardware

Браузер:

просмотр видео
выбор цели
настройка режима
запуск/останов AUTO
показ телеметрии

2. Серверный tracker
Нужно сделать

    стабильные track_id;

    удержание выбранного объекта;

    запрет самовольной подмены цели;

    состояние TRACKING / LOST / REACQUIRE / IDLE;

    серверный API:

GET  /api/tracker/state
POST /api/tracker/select
POST /api/tracker/clear
POST /api/tracker/config

Логика

YOLO bbox
  → IoU / distance association
  → stable track_id
  → selected_track_id
  → target state

3. ROI / tiled detection
Задача

Улучшить распознавание дальних машин без увеличения модели до 1024/1280.
Режимы

full_frame     — весь кадр
roi            — один выбранный участок
multi_roi      — несколько участков
tiled          — нарезка кадра на зоны
hybrid         — ROI каждый кадр + full frame раз в N кадров

Конфиг

{
  "detection_mode": "hybrid",
  "rois": [
    {
      "id": "parking",
      "enabled": true,
      "x": 500,
      "y": 220,
      "w": 900,
      "h": 500,
      "every_n_frames": 1,
      "classes": ["car", "truck", "bus", "person"]
    },
    {
      "id": "full_scan",
      "enabled": true,
      "x": 0,
      "y": 0,
      "w": 1920,
      "h": 1080,
      "every_n_frames": 10
    }
  ]
}

Важно

После ROI inference координаты bbox надо возвращать в координаты полного кадра.
4. Zoom calibration через AprilTag
Исходные данные

    камера: Canon Legria HF M306E;

    zoom управляется сервоприводом SG90;

    zoom механический/аналоговый;

    есть AprilTag размером 160 × 160 мм;

    servo двигает zoom-ползунок:

        далеко от центра — zoom быстро едет;

        близко к центру — zoom медленно ползёт;

        значит нужна модель servo_command + time → zoom_position.

5. Профиль zoom

Нужно сохранять профиль:

{
  "camera": "Canon Legria HF M306E",
  "zoom_drive": "SG90 servo",
  "marker_size_m": 0.160,
  "image_width": 1920,
  "image_height": 1080,
  "samples": [
    {
      "zoom_norm": 0.0,
      "servo_cmd": -34,
      "hold_ms": 0,
      "tag_px_w": 80,
      "tag_px_h": 80,
      "fx_px": 900,
      "fy_px": 910,
      "pid_scale": 1.0
    },
    {
      "zoom_norm": 0.5,
      "servo_cmd": 12,
      "hold_ms": 1500,
      "tag_px_w": 160,
      "tag_px_h": 160,
      "fx_px": 1800,
      "fy_px": 1820,
      "pid_scale": 0.5
    },
    {
      "zoom_norm": 1.0,
      "servo_cmd": 34,
      "hold_ms": 4000,
      "tag_px_w": 320,
      "tag_px_h": 320,
      "fx_px": 3600,
      "fy_px": 3650,
      "pid_scale": 0.25
    }
  ]
}

6. Zoom calibration procedure
Этапы

1. Перевести zoom в известное крайнее положение WIDE.
2. Считать zoom_norm = 0.0.
3. Найти AprilTag 160 мм.
4. Измерить размер тега в пикселях.
5. Сохранить sample.
6. Подать servo-команду zoom-in на заданное время.
7. Подождать стабилизацию.
8. Снова найти AprilTag.
9. Сохранить следующий sample.
10. Повторить до TELE.
11. Сохранить zoom_profile.json.

API

POST /api/zoom_profile/calibrate/start
POST /api/zoom_profile/calibrate/stop
GET  /api/zoom_profile/state
GET  /api/zoom_profile
POST /api/zoom_profile

7. Zoom-aware PID

PID должен работать не просто по пиксельной ошибке:

errX = target_x - center_x

А по угловой ошибке:

angle_x = atan((target_x - center_x) / fx_px)
angle_y = atan((target_y - center_y) / fy_px)

Где fx_px / fy_px берутся из zoom profile.

Так управление будет адекватным на wide и на сильном zoom.
8. Серверный autopilot

Добавить отдельный процесс:

ptz_autopilot

Он делает:

1. читает /api/tracker/state
2. читает /api/zoom_profile
3. считает angular PID
4. отправляет J-команды в ws_uart_bridge
5. ведёт состояние AUTO / ASSIST / MANUAL / SAFE

launcher.sh потом будет запускать:

mjpeg_rknn_http
ws_uart_bridge
ptz_autopilot

9. Режимы работы
MANUAL

Пользователь сам управляет pan/tilt/zoom.
ASSIST

Система показывает цель и может мягко помогать, но пользователь остаётся главным.
AUTO

Сервер сам держит выбранную цель.
REACQUIRE

Если цель потеряна:

короткая потеря → hold last direction
длинная потеря → scan
таймаут → stop / assist / manual

10. Порядок реализации
Этап 1

    исправить dt;

    убедиться, что stable track_id работает;

    убедиться, что выбранная цель не подменяется.

Этап 2

    перенести selected target на сервер;

    добавить /api/tracker/state;

    браузер должен только выбирать цель, а не считать PID.

Этап 3

    добавить ROI detection;

    сделать remap bbox координат;

    добавить hybrid detection mode.

Этап 4

    реализовать AprilTag zoom calibration;

    сохранить zoom_profile.json.

Этап 5

    сделать zoom-aware angular PID.

Этап 6

    добавить ptz_autopilot;

    подключить его в launcher.sh.

Этап 7

    дообучение модели под парковку / дальние машины / конкретный ракурс.


Ключевое уточнение по zoom: у тебя не “абсолютный zoom position”, а **механический привод без обратной связи**. Значит профиль должен описывать не только `zoom_norm`, но и связь:

```text
servo_command + hold_time → изменение zoom_norm

А AprilTag нужен как внешний измеритель фактического результата.

