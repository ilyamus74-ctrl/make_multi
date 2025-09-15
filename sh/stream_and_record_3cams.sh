#!/usr/bin/env bash
set -euo pipefail

CONFIG="${1:-config.json}"
DURATION="${DURATION:-120}"          # 120s = 2 минуты
HOST_IP="${HOST_IP:-0.0.0.0}"        # адрес для бинда HTTP
TMPDIR="${TMPDIR:-/tmp}"

need(){ command -v "$1" >/dev/null 2>&1 || { echo "[-] need $1"; exit 1; }; }
need jq
need ffmpeg
need mkfifo

STAMP="$(date +%Y%m%d_%H%M%S)"
COUNT="$(jq '.cameras | length' "$CONFIG")"
[[ "$COUNT" -ge 1 ]] || { echo "[-] cameras[] empty in $CONFIG"; exit 1; }

echo "[i] starting $COUNT cameras for ${DURATION}s…"

# держим PID'ы процессов
writers=()
servers=()
fifos=()

cleanup(){
  echo; echo "[i] stopping…"
  for p in "${writers[@]:-}"; do kill -TERM "$p" 2>/dev/null || true; done
  for p in "${servers[@]:-}"; do kill -TERM "$p" 2>/dev/null || true; done
  sleep 0.5
  for f in "${fifos[@]:-}"; do [[ -p "$f" ]] && rm -f "$f"; done
}
trap cleanup EXIT

for idx in $(seq 0 $((COUNT-1))); do
  DEV=$(jq -r ".cameras[$idx].device" "$CONFIG")
  ID=$(jq -r ".cameras[$idx].id" "$CONFIG")
  PORT=$(jq -r ".cameras[$idx].det_port" "$CONFIG")
  W=$(jq -r ".cameras[$idx].preferred.w" "$CONFIG")
  H=$(jq -r ".cameras[$idx].preferred.h" "$CONFIG")
  FPS=$(jq -r ".cameras[$idx].preferred.fps" "$CONFIG")
  PIXFMT=$(jq -r ".cameras[$idx].preferred.pixfmt" "$CONFIG")

  [[ -z "$DEV" || "$DEV" == "null" || -z "$ID" || -z "$PORT" ]] && { echo "[-] bad camera idx $idx"; exit 1; }

  OUTDIR="cam${ID}/${STAMP}"
  OUTFILE="${OUTDIR}/out.avi"
  mkdir -p "$OUTDIR"

  FIFO="${TMPDIR}/cam${ID}_${STAMP}.mjpg"
  [[ -p "$FIFO" ]] && rm -f "$FIFO"
  mkfifo "$FIFO"
  fifos+=("$FIFO")

  # входной формат из конфига
  INPIX="mjpeg"
  [[ "${PIXFMT^^}" == "MJPG" || "${PIXFMT^^}" == "MJPEG" ]] || INPIX="yuyv422"

  echo "[i] cam${ID}: $DEV ${W}x${H}@${FPS} (${PIXFMT}) -> file:${OUTFILE} & http://${HOST_IP}:${PORT}/stream.mjpg"

  # 1) HTTP сервер из FIFO (отдельный ffmpeg c -listen 1)
  ffmpeg -hide_banner -loglevel warning -nostdin \
    -fflags nobuffer -re \
    -f mjpeg -i "$FIFO" \
    -f mpjpeg -q:v 5 -listen 1 "http://${HOST_IP}:${PORT}/stream.mjpg" \
    & servers+=("$!")

  # 2) Писатель: читает V4L2 один раз, кодирует MJPEG, tee -> файл + FIFO
  ffmpeg -hide_banner -loglevel warning -nostdin \
    -f v4l2 -input_format "$INPIX" -framerate "$FPS" -video_size "${W}x${H}" \
    -thread_queue_size 512 -rtbufsize 256M -i "$DEV" \
    -map 0:v -c:v mjpeg -q:v 5 \
    -t "$DURATION" \
    -f tee \
    "[onfail=ignore:use_fifo=1:f=avi]${OUTFILE}|[onfail=ignore:use_fifo=1:f=mjpeg]file:${FIFO}" \
    & writers+=("$!")

  # даём серверу подняться
  sleep 0.4
done

echo
echo "[i] open streams in browser:"
for idx in $(seq 0 $((COUNT-1))); do
  ID=$(jq -r ".cameras[$idx].id" "$CONFIG")
  PORT=$(jq -r ".cameras[$idx].det_port" "$CONFIG")
  echo "    cam${ID}: http://${HOST_IP}:${PORT}/stream.mjpg"
done
echo

# ждём окончания записи
fail=0
for p in "${writers[@]}"; do wait "$p" || fail=1; done

# после окончания записи завершаем HTTP-сервера
for p in "${servers[@]}"; do kill -TERM "$p" 2>/dev/null || true; done

echo
echo "[i] files saved:"
find cam*/"${STAMP}" -maxdepth 1 -type f -name "out.avi" -printf "  %p\n" 2>/dev/null || true
exit $fail
