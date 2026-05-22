# new_yolo8: MJPEG + серверная детекция YOLOv8 (RKNN, Rock5B+)


- `mjpeg_gst_http.cpp` — HTTP MJPEG сервер.
- Детекция выполняется **на стороне сервера** в потоке обработки входящего видео (до отдачи в веб).
- В веб-интерфейсе есть:
  - кнопка **«Распознать»** (ON/OFF),
  - список доступных `*.rknn` моделей,
  - кнопка применения выбранной модели.

## Где должны лежать модели

По умолчанию сервер ищет модели в:

- `new_yolo8/model_rknn/`
- `new_yolo8/models/`
- `model_rknn/`

Можно указать свои пути:

- `--model-dir <dir>` — каталог со списком моделей,
- `--model <path_to_model.rknn>` — явно выбрать модель на старте.

## Зависимости

- GStreamer: `gstreamer-1.0`, `gstreamer-app-1.0`
- OpenCV (imgcodecs + imgproc)
- RKNN runtime (`librknnrt.so`)
- исходники/заголовки проекта: `yolov8.h`, `postprocess.h`, `postprocess.cc`, `rknpu2/yolov8.cc`, `include/*`

## Важный файл лейблов

Для `postprocess` нужен список классов, по умолчанию:

- `model/coco_80_labels_list.txt`

Путь можно изменить флагом `--labels`.

## Сборка через CMake (компиляция делегирована в `build_mjpeg_rknn.sh`)

```bash
cmake -S . -B build
cmake --build build
```

CMake-цель `mjpeg_rknn_http` не компилирует напрямую, а запускает `build_mjpeg_rknn.sh`. Сам скрипт вызывает `gcc/g++` и кладёт результат в каталог сборки (`BUILD_DIR`, по умолчанию `.build`).

## Запуск

```bash
./mjpeg_gst_http --dev /dev/video0 --port 8080 --width 1920 --height 1080 --fps 25 --jpeg 80 --deinterlace \
  --labels model/coco_80_labels_list.txt --model-dir new_yolo8/model_rknn
```

Открыть: `http://<ip>:8080/`.

##сборка для меня
g++ -O2 -std=c++17 mjpeg_gst_http.cpp -o mjpeg_gst_http   $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0) -lpthread
В этой папке собран отдельный минимальный вариант для машинного зрения на Rock5B+:
sudo ./mjpeg_gst_http --dev /dev/video0 --port 8080 --width 1920 --height 1080 --fps 25 --jpeg 80 --deinterlace

## Быстрый запуск двух сервисов одним скриптом (launcher.sh)

Для режима «одной команды» используйте:

```bash
./launcher.sh
```


Опционально можно создать `launcher.cfg` (см. шаблон `launcher.cfg.example`) — тогда лимиты и порты будут подхватываться автоматически без длинного префикса `ENV=...`.

Скрипт запускает:

- `mjpeg_gst_http` (видео/детекция) на `MJPEG_PORT` (по умолчанию `8080`),
- `web/ws_uart_bridge` (WS↔UART) на `WS_PORT` (по умолчанию `8765`).

Логи пишутся в `.logs/`:

- `.logs/mjpeg.log`
- `.logs/bridge.log`

### Переопределение параметров через ENV

Пример:

```bash
VIDEO_DEV=/dev/video2 UART_DEV=/dev/ttyS3 MJPEG_PORT=8081 WS_PORT=9001 ./launcher.sh
```

Поддерживаемые переменные:

- `MJPEG_BIN`, `BRIDGE_BIN`
- `VIDEO_DEV`, `MJPEG_PORT`, `WIDTH`, `HEIGHT`, `FPS`, `JPEG_QUALITY`, `DEINTERLACE`
- `LABELS`, `MODEL_DIR`
- `CMD_MAX_PAN`, `CMD_MAX_TILT`, `CMD_MAX_ZOOM` (ограничения команд для механики, по умолчанию 20/20/12)
- `UART_DEV`, `UART_BAUD`, `WS_PORT`
- `LOG_DIR`, `MJPEG_LOG`, `BRIDGE_LOG`

### Миграция к systemd (вариант A)

`launcher.sh` специально сделан как переходный вариант:

- параметры вынесены в ENV,
- процессы независимы,
- есть корректное завершение по `SIGINT/SIGTERM`.

При переходе на `systemd` эти же переменные удобно перенести в `EnvironmentFile=` и разнести на два unit-файла (`mjpeg.service` и `bridge.service`) с объединяющим `target`.
