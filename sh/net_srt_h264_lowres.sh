#!/usr/bin/env bash
set -euo pipefail

CONFIG="${1:-config.json}"
DURATION="${DURATION:-120}"          # сек
BASE_PORT="${BASE_PORT:-9000}"
OUTROOT="/dev/shm"; [[ -d $OUTROOT ]] || OUTROOT="/tmp"

# цели для СЕТЕВОЙ ветки (сильно разгружают CPU)
NET_W="${NET_W:-640}"
NET_H="${NET_H:-480}"
NET_FPS="${NET_FPS:-10}"
CAP_FPS="${CAP_FPS:-15}"            # FPS захвата с камеры

need(){ command -v "$1" >/dev/null 2>&1 || { echo "need $1"; exit 1; }; }
need jq; need ffmpeg

N=$(jq '.cameras | length' "$CONFIG"); [[ $N -ge 1 ]] || { echo "cameras[] empty"; exit 1; }
STAMP=$(date +%Y%m%d_%H%M%S)

# детект H.264 энкодера
ENC_LIST="$(ffmpeg -hide_banner -encoders 2>/dev/null)"
if echo "$ENC_LIST" | grep -Eiq '^\s*V.*h264_v4l2m2m\b'; then
  H264_IMPL="h264_v4l2m2m"
elif echo "$ENC_LIST" | grep -Eiq '^\s*V.*libx264\b'; then
  H264_IMPL="libx264"
else
  echo "[-] no H.264 encoder in ffmpeg -encoders"; exit 1
fi
echo "[i] using H.264 encoder: $H264_IMPL"

HAS_SRT=0; ffmpeg -protocols 2>/dev/null | grep -qw srt && HAS_SRT=1

pids=()
for i in $(seq 0 $((N-1))); do
  DEV=$(jq -r ".cameras[$i].device" "$CONFIG")
  ID=$(jq -r  ".cameras[$i].id" "$CONFIG")
  W=$(jq -r   ".cameras[$i].preferred.w" "$CONFIG")
  H=$(jq -r   ".cameras[$i].preferred.h" "$CONFIG")
  FPS=$(jq -r ".cameras[$i].preferred.fps" "$CONFIG")
  PIX=$(jq -r ".cameras[$i].preferred.pixfmt" "$CONFIG")
  [[ -z "$DEV" || "$DEV" == "null" || -z "$ID" ]] && { echo "bad camera $i"; exit 1; }

  PORT=$((BASE_PORT + i))
  OUTDIR="${OUTROOT}/cam${ID}/${STAMP}"; mkdir -p "$OUTDIR"
  OUTFILE="${OUTDIR}/out.avi"

  # входной формат (если не MJPG — возьмём yuyv422)
  INPIX="mjpeg"; [[ "${PIX^^}" == "MJPG" || "${PIX^^}" == "MJPEG" ]] || INPIX="yuyv422"

  # сетевой URL
  if [[ $HAS_SRT -eq 1 ]]; then
    NET_URL="srt://0.0.0.0:${PORT}?mode=listener&latency=80&transtype=live"
  else
    NET_URL="udp://0.0.0.0:${PORT}?listen=1&pkt_size=1316&fifo_size=1000000&overrun_nonfatal=1"
  fi

  # параметры кодека
  if [[ "$H264_IMPL" == "h264_v4l2m2m" ]]; then
    NET_CODEC=(-c:v h264_v4l2m2m -pix_fmt nv12 -b:v 2M -maxrate 2M -bufsize 1M -g $((NET_FPS*2)) -bf 0 -tune zerolatency)
  else
    NET_CODEC=(-c:v libx264 -preset ultrafast -tune zerolatency -x264-params "keyint=$((NET_FPS*2)):scenecut=0" -g $((NET_FPS*2)) -bf 0 -threads 2)
  fi

  echo "[i] cam${ID}: $DEV ${W}x${H}@${CAP_FPS} (${PIX}) -> file:${OUTFILE}; NET:${NET_URL} ${NET_W}x${NET_H}@${NET_FPS}"
  # Ключевые флаги:
  #  -use_libv4l2 1 : обходит глюк VIDIOC_G_INPUT на некоторых UVC
  #  -ts mono2abs + +genpts : стабильные метки времени
  ffmpeg -hide_banner -nostdin -loglevel error \
    -fflags +genpts -flags low_delay -threads 1 \
    -f v4l2 -use_libv4l2 1 -ts mono2abs \
    -input_format "$INPIX" -framerate "$CAP_FPS" -video_size "${W}x${H}" -rtbufsize 32M \
    -i "$DEV" \
    -filter_complex "[0:v]split=2[vfile][vtmp];[vtmp]fps=${NET_FPS},scale=${NET_W}:${NET_H},format=nv12[vnet]" \
    -map "[vfile]" -vsync vfr  -c:v mjpeg -q:v 6 \
    -map "[vnet]"  -vsync cfr  -r "${NET_FPS}" "${NET_CODEC[@]}" \
    -t "$DURATION" \
    -f avi "$OUTFILE" \
    -f mpegts -muxdelay 0 -muxpreload 0 "$NET_URL" \
    & pids+=($!)

  sleep 0.2
done

echo; echo "[i] viewer (client):"
for i in $(seq 0 $((N-1))); do
  PORT=$((BASE_PORT + i))
  if [[ $HAS_SRT -eq 1 ]]; then
    echo "  ffplay -fflags nobuffer -flags low_delay -probesize 32k -an -sync video srt://<IP>:${PORT}?mode=caller&latency=80"
  else
    echo "  ffplay -fflags nobuffer -flags low_delay -probesize 32k -an -sync video udp://<IP>:${PORT}"
  fi
done

fail=0; for p in "${pids[@]}"; do wait "$p" || fail=1; done
echo; echo "[i] files:"; find "$OUTROOT" -maxdepth 3 -type f -path "*/${STAMP}/out.avi" -print
exit $fail
