#!/usr/bin/env bash
set -euo pipefail

CONFIG="${1:-config.json}"
DURATION="${DURATION:-120}"   # секунд (по умолчанию 120 = 2 минуты)
HOST_IP="${HOST_IP:-0.0.0.0}" # куда биндить стримы (по умолчанию на все интерфейсы)

need() { command -v "$1" >/dev/null 2>&1 || { echo "[-] required tool '$1' not found"; exit 1; }; }
need jq
need ffmpeg

if [[ ! -f "$CONFIG" ]]; then
  echo "[-] no $CONFIG"; exit 1
fi

ts() { date +%Y%m%d_%H%M%S; }
STAMP="$(ts)"

# Список камер из config.json
COUNT=$(jq '.cameras | length' "$CONFIG")
if [[ "$COUNT" -eq 0 ]]; then
  echo "[-] cameras[] is empty in $CONFIG"; exit 1
fi

pids=()

echo "[i] will start $COUNT camera streams & 2-min recordings..."
for idx in $(seq 0 $((COUNT-1))); do
  DEV=$(jq -r ".cameras[$idx].device" "$CONFIG")
  ID=$(jq -r ".cameras[$idx].id" "$CONFIG")
  PORT=$(jq -r ".cameras[$idx].det_port" "$CONFIG")
  W=$(jq -r ".cameras[$idx].preferred.w" "$CONFIG")
  H=$(jq -r ".cameras[$idx].preferred.h" "$CONFIG")
  FPS=$(jq -r ".cameras[$idx].preferred.fps" "$CONFIG")
  PIXFMT=$(jq -r ".cameras[$idx].preferred.pixfmt" "$CONFIG")

  [[ "$DEV" == "null" || "$ID" == "null" || "$PORT" == "null" ]] && { echo "[-] bad camera entry at index $idx"; exit 1; }

  # Папка записи: cam<ID>/<STAMP>/
  OUTDIR="cam${ID}/${STAMP}"
  mkdir -p "$OUTDIR"
  OUTFILE="${OUTDIR}/out.avi"

  # Подскажем URL
  echo "[i] cam${ID}: device=${DEV} ${W}x${H}@${FPS} pixfmt=${PIXFMT}  ->  http://${HOST_IP}:${PORT}/stream.mjpg"
  echo "    recording -> ${OUTFILE}"

  # Для v4l2 MJPG
  INPIX="mjpeg"
  if [[ "${PIXFMT^^}" != "MJPG" && "${PIXFMT^^}" != "MJPEG" ]]; then
    #fallback (камера не MJPG) — всё равно тащим; ffmpeg перекодирует для http, а в файл можно и в mjpeg
    INPIX="yuyv422"
  fi

  # Один ffmpeg на камеру:
  #  - читает V4L2
  #  - без перекодирования кладёт MJPEG в AVI на диск
  #  - параллельно даёт HTTP MJPEG (mpjpeg muxer) на det_port
  #  - останавливается через -t $DURATION
  ffmpeg -hide_banner -loglevel warning -nostdin \
    -f v4l2 -input_format "${INPIX}" -framerate "${FPS}" -video_size "${W}x${H}" -thread_queue_size 512 -i "${DEV}" \
    -map 0:v -c:v mjpeg -q:v 5 \
    -t "${DURATION}" \
    -f tee "[onfail=ignore:use_fifo=1:f=avi]${OUTFILE}|[onfail=ignore:use_fifo=1:f=mpjpeg]http://${HOST_IP}:${PORT}/stream.mjpg?listen=1" \
    & pids+=($!)

  # Небольшой сдвиг, чтобы порты успели привязаться
  sleep 0.3
done

echo
echo "[i] Streams are up. Open in browser (каждая камера в своём окне/вкладке):"
for idx in $(seq 0 $((COUNT-1))); do
  ID=$(jq -r ".cameras[$idx].id" "$CONFIG")
  PORT=$(jq -r ".cameras[$idx].det_port" "$CONFIG")
  echo "    cam${ID}: http://${HOST_IP}:${PORT}/stream.mjpg"
done
echo
echo "[i] Recording will run for ${DURATION}s. Press CTRL+C to abort early."

# Ждём завершения всех ffmpeg
fail=0
for pid in "${pids[@]}"; do
  if ! wait "$pid"; then fail=1; fi
done

echo
echo "[i] Done. Files recorded:"
find cam*/"${STAMP}" -maxdepth 1 -type f -name "out.avi" -printf "  %p\n" 2>/dev/null || true

exit $fail
