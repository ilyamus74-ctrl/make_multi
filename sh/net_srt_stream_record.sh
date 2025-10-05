#!/usr/bin/env bash
set -euo pipefail

CONFIG="${1:-config.json}"
DURATION="${DURATION:-120}"         # секунд (2 минуты по умолчанию)
SRT_BASE="${SRT_BASE:-9000}"        # порты: 9000, 9001, 9002,...
OUTROOT="/dev/shm"                  # RAM; если нет /dev/shm, скрипт сам уйдет в /tmp

need(){ command -v "$1" >/dev/null 2>&1 || { echo "need $1"; exit 1; }; }
need jq; need ffmpeg

[[ -d "$OUTROOT" ]] || OUTROOT="/tmp"
STAMP="$(date +%Y%m%d_%H%M%S)"
N="$(jq '.cameras | length' "$CONFIG")"
[[ "$N" -ge 1 ]] || { echo "cameras[] empty"; exit 1; }

# проверим поддержку SRT и H.264 аппаратного энкодера
HAS_SRT=0; ffmpeg -protocols 2>/dev/null | grep -qw srt && HAS_SRT=1
[[ "$HAS_SRT" -eq 1 ]] || echo "[!] ffmpeg без SRT. Могу переключить на UDP/TS (скажи, если нужно)."
HAS_V4L2M2M=0; ffmpeg -encoders 2>/dev/null | grep -qw h264_v4l2m2m && HAS_V4L2M2M=1

pids=()

for i in $(seq 0 $((N-1))); do
  DEV=$(jq -r ".cameras[$i].device" "$CONFIG")
  ID=$(jq -r ".cameras[$i].id" "$CONFIG")
  W=$(jq -r  ".cameras[$i].preferred.w" "$CONFIG")
  H=$(jq -r  ".cameras[$i].preferred.h" "$CONFIG")
  FPS=$(jq -r ".cameras[$i].preferred.fps" "$CONFIG")
  PIX=$(jq -r ".cameras[$i].preferred.pixfmt" "$CONFIG")

  [[ -z "$DEV" || "$DEV" == "null" || -z "$ID" ]] && { echo "bad camera entry $i"; exit 1; }

  PORT=$((SRT_BASE + i))
  OUTDIR="${OUTROOT}/cam${ID}/${STAMP}"
  OUTFILE="${OUTDIR}/out.avi"
  mkdir -p "$OUTDIR"

  INPIX="mjpeg"; [[ "${PIX^^}" == "MJPG" || "${PIX^^}" == "MJPEG" ]] || INPIX="yuyv422"

  # Выбор сетевого кодека
  if [[ "$HAS_V4L2M2M" -eq 1 ]]; then
    NET_CODEC=(-c:v h264_v4l2m2m -pix_fmt nv12 -b:v 4M -maxrate 4M -bufsize 2M -g $((FPS*2)) -bf 0 -tune zerolatency)
  else
    NET_CODEC=(-c:v libx264 -preset ultrafast -tune zerolatency -x264-params "keyint=$((FPS*2)):scenecut=0" -g $((FPS*2)) -bf 0)
  fi

  # Куда стримим (SRT listener на этом устройстве)
  if [[ "$HAS_SRT" -eq 1 ]]; then
    NET_URL="srt://0.0.0.0:${PORT}?mode=listener&latency=60&rcvbuf=2000000&sndbuf=2000000"
    NET_FMT=("mpegts" -muxdelay 0 -muxpreload 0)
  else
    # запасной вариант (скажи, если хочешь принудительно UDP)
    NET_URL="udp://0.0.0.0:${PORT}?listen=1&fifo_size=1000000&overrun_nonfatal=1&pkt_size=1316"
    NET_FMT=("mpegts" -muxdelay 0 -muxpreload 0)
  fi

  echo "[i] cam${ID}: $DEV ${W}x${H}@${FPS} (${PIX}) → file:${OUTFILE}; NET ${NET_URL}"

  # Один ffmpeg на камеру: читаем 1 раз, split на файл (MJPEG) и сеть (H.264)
  ffmpeg -hide_banner -nostdin -loglevel error \
    -fflags nobuffer -flags low_delay -threads 1 \
    -f v4l2 -input_format "$INPIX" -framerate "$FPS" -video_size "${W}x${H}" \
    -thread_queue_size 64 -rtbufsize 16M -use_wallclock_as_timestamps 1 \
    -i "$DEV" \
    -filter_complex "[0:v]split=2[vfile][vnet]" \
    -map "[vfile]" -vsync drop -c:v mjpeg -q:v 5 \
    -map "[vnet]"  -vsync drop "${NET_CODEC[@]}" \
    -t "$DURATION" \
    -f avi "$OUTFILE" \
    -f "${NET_FMT[@]}" "$NET_URL" \
    & pids+=($!)

  sleep 0.3
done

echo
echo "[i] Вьювер с ДРУГОГО ПК (замени <IP_устройства>):"
for i in $(seq 0 $((N-1))); do
  ID=$(jq -r ".cameras[$i].id" "$CONFIG")
  PORT=$((SRT_BASE + i))
  if [[ "$HAS_SRT" -eq 1 ]]; then
    echo "  ffplay -fflags nobuffer -flags low_delay -probesize 32k -an -sync video srt://<IP_устройства>:${PORT}?mode=caller&latency=60"
  else
    echo "  ffplay -fflags nobuffer -flags low_delay -probesize 32k -an -sync video udp://<IP_устройства>:${PORT}"
  fi
done
echo
echo "[i] Ждём ${DURATION}s..."
fail=0
for p in "${pids[@]}"; do wait "$p" || fail=1; done

echo
echo "[i] Готово. Файлы (RAM):"
find "${OUTROOT}" -maxdepth 3 -type f -path "*/${STAMP}/out.avi" -print
exit $fail
