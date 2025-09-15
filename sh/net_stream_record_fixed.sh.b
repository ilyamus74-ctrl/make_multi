#!/usr/bin/env bash
set -euo pipefail

CONFIG="${1:-config.json}"
DURATION="${DURATION:-120}"         # 2 минуты
BASE_PORT="${BASE_PORT:-9000}"      # базовый порт (для SRT/UDP)
OUTROOT="/dev/shm"
[[ -d "$OUTROOT" ]] || OUTROOT="/tmp"

need(){ command -v "$1" >/dev/null 2>&1 || { echo "need $1"; exit 1; }; }
need jq; need ffmpeg

STAMP="$(date +%Y%m%d_%H%M%S)"
N="$(jq '.cameras | length' "$CONFIG")"
[[ "$N" -ge 1 ]] || { echo "cameras[] empty"; exit 1; }

# возможности
HAS_SRT=0;  ffmpeg -protocols 2>/dev/null | grep -qw srt && HAS_SRT=1
HAS_V4L2=0; ffmpeg -encoders 2>/dev/null   | grep -qw h264_v4l2m2m && HAS_V4L2=1
HAS_X264=0; ffmpeg -encoders 2>/dev/null   | grep -qw libx264       && HAS_X264=1

pids=()

for i in $(seq 0 $((N-1))); do
  DEV=$(jq -r ".cameras[$i].device" "$CONFIG")
  ID=$(jq -r  ".cameras[$i].id" "$CONFIG")
  W=$(jq -r   ".cameras[$i].preferred.w" "$CONFIG")
  H=$(jq -r   ".cameras[$i].preferred.h" "$CONFIG")
  FPS=$(jq -r ".cameras[$i].preferred.fps" "$CONFIG")
  PIX=$(jq -r ".cameras[$i].preferred.pixfmt" "$CONFIG")

  [[ -z "$DEV" || "$DEV" == "null" || -z "$ID" ]] && { echo "bad camera entry $i"; exit 1; }

  PORT=$((BASE_PORT + i))
  OUTDIR="${OUTROOT}/cam${ID}/${STAMP}"
  OUTFILE="${OUTDIR}/out.avi"
  mkdir -p "$OUTDIR"

  # входной формат V4L2
  INPIX="mjpeg"; [[ "${PIX^^}" == "MJPG" || "${PIX^^}" == "MJPEG" ]] || INPIX="yuyv422"

  # сетевой контейнер/URL
  if [[ "$HAS_SRT" -eq 1 ]]; then
    NET_URL="srt://0.0.0.0:${PORT}?mode=listener&latency=80&transtype=live"
  else
    NET_URL="udp://0.0.0.0:${PORT}?listen=1&pkt_size=1316&fifo_size=1000000&overrun_nonfatal=1"
  fi

  # выбираем кодек для сети
  if   [[ "$HAS_V4L2" -eq 1 ]]; then
    NET_CODEC=(-c:v h264_v4l2m2m -pix_fmt nv12 -b:v 4M -maxrate 4M -bufsize 2M -g $((FPS*2)) -bf 0 -tune zerolatency)
  elif [[ "$HAS_X264" -eq 1 ]]; then
    NET_CODEC=(-c:v libx264 -preset ultrafast -tune zerolatency -x264-params "keyint=$((FPS*2)):scenecut=0" -g $((FPS*2)) -bf 0)
  else
    # запасной путь: MPEG-2 с правильными PTS (CFR + genpts), чтобы не падал
    NET_CODEC=(-c:v mpeg2video -qscale:v 2 -g $((FPS*2)) -bf 0 -flags2 +fast -r "${FPS}")
  fi

  echo "[i] cam${ID}: ${DEV} ${W}x${H}@${FPS} (${PIX}) → file:${OUTFILE}; NET ${NET_URL}"

  # один вход, split на файл (MJPEG) и сеть (H.26x или mpeg2)
  # ВАЖНО: генерим PTS и фиксируем сетевой FPS, убраны wallclock/copyts.
  ffmpeg -hide_banner -nostdin -loglevel error \
    -fflags +genpts -flags low_delay -thread_queue_size 64 -threads 1 \
    -f v4l2 -input_format "$INPIX" -framerate "$FPS" -video_size "${W}x${H}" -rtbufsize 16M \
    -i "$DEV" \
    -filter_complex "[0:v]split=2[vfile][vnet]" \
    -map "[vfile]" -vsync vfr  -c:v mjpeg -q:v 5 \
    -map "[vnet]"  -vsync cfr  -r "${FPS}" "${NET_CODEC[@]}" \
    -t "$DURATION" \
    -f avi "$OUTFILE" \
    -f mpegts -muxdelay 0 -muxpreload 0 "$NET_URL" \
    & pids+=($!)

  sleep 0.2
done

echo
echo "[i] Вьювер с другого ПК (замени <IP>):"
for i in $(seq 0 $((N-1))); do
  ID=$(jq -r ".cameras[$i].id" "$CONFIG")
  PORT=$((BASE_PORT + i))
  if [[ "$HAS_SRT" -eq 1 ]]; then
    echo "  ffplay -fflags nobuffer -flags low_delay -probesize 32k -an -sync video srt://<IP>:${PORT}?mode=caller&latency=80"
  else
    echo "  ffplay -fflags nobuffer -flags low_delay -probesize 32k -an -sync video udp://<IP>:${PORT}"
  fi
done

echo
echo "[i] waiting ${DURATION}s…"
fail=0
for p in "${pids[@]}"; do wait "$p" || fail=1; done

echo
echo "[i] files:"
find "${OUTROOT}" -maxdepth 3 -type f -path "*/${STAMP}/out.avi" -print
exit $fail