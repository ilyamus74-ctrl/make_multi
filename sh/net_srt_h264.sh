#!/usr/bin/env bash
set -euo pipefail

CONFIG="${1:-config.json}"
DURATION="${DURATION:-120}"
BASE_PORT="${BASE_PORT:-9000}"
OUTROOT="/dev/shm"; [[ -d $OUTROOT ]] || OUTROOT="/tmp"
FFMPEG="${FFMPEG:-ffmpeg}"

need(){ command -v "$1" >/dev/null 2>&1 || { echo "need $1"; exit 1; }; }
need jq; need "$FFMPEG"

N=$(jq '.cameras | length' "$CONFIG"); [[ $N -ge 1 ]] || { echo "cameras[] empty"; exit 1; }
STAMP=$(date +%Y%m%d_%H%M%S)

# Детект энкодеров (подавляем баннер, матчим построчно)
ENC_LIST="$("$FFMPEG" -hide_banner -encoders 2>/dev/null | sed 's/\x1B\[[0-9;]*[A-Za-z]//g')"
if echo "$ENC_LIST" | grep -Eiq '^\s*V.*h264_v4l2m2m\b'; then
  H264_IMPL="h264_v4l2m2m"
elif echo "$ENC_LIST" | grep -Eiq '^\s*V.*libx264\b'; then
  H264_IMPL="libx264"
else
  echo "[-] H.264 encoder not found in ffmpeg -encoders"
  echo "$ENC_LIST" | sed -n '1,40p'
  exit 1
fi
echo "[i] H.264 encoder selected: $H264_IMPL"

HAS_SRT=0; "$FFMPEG" -protocols 2>/dev/null | grep -qw srt && HAS_SRT=1
[[ $HAS_SRT -eq 1 ]] || echo "[!] ffmpeg without SRT; will fall back to UDP (9000+)."

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

  INPIX="mjpeg"; [[ "${PIX^^}" == "MJPG" || "${PIX^^}" == "MJPEG" ]] || INPIX="yuyv422"

  if [[ $HAS_SRT -eq 1 ]]; then
    NET_URL="srt://0.0.0.0:${PORT}?mode=listener&latency=80&transtype=live"
  else
    NET_URL="udp://0.0.0.0:${PORT}?listen=1&pkt_size=1316&fifo_size=1000000&overrun_nonfatal=1"
  fi

  # Параметры H.264
  if [[ "$H264_IMPL" == "h264_v4l2m2m" ]]; then
    NET_CODEC=(-c:v h264_v4l2m2m -pix_fmt nv12 -b:v 4M -maxrate 4M -bufsize 2M -g $((FPS*2)) -bf 0 -tune zerolatency)
  else
    NET_CODEC=(-c:v libx264 -preset ultrafast -tune zerolatency -x264-params "keyint=$((FPS*2)):scenecut=0" -g $((FPS*2)) -bf 0)
  fi

  echo "[i] cam${ID}: $DEV ${W}x${H}@${FPS} → file:${OUTFILE}; NET:${NET_URL}; codec:${H264_IMPL}"

  "$FFMPEG" -hide_banner -loglevel error -fflags +genpts -flags low_delay -threads 1 \
    -f v4l2 -input_format "$INPIX" -framerate "$FPS" -video_size "${W}x${H}" -rtbufsize 16M \
    -i "$DEV" \
    -filter_complex "[0:v]split=2[vf][vn]" \
    -map "[vf]" -vsync vfr -c:v mjpeg -q:v 5 \
    -map "[vn]" -vsync cfr -r "$FPS" "${NET_CODEC[@]}" \
    -t "$DURATION" \
    -f avi "$OUTFILE" \
    -f mpegts -muxdelay 0 -muxpreload 0 "$NET_URL" \
    & pids+=($!)
  sleep 0.3
done

echo; echo "[i] viewer from client:"
for i in $(seq 0 $((N-1))); do
  ID=$(jq -r ".cameras[$i].id" "$CONFIG"); PORT=$((BASE_PORT + i))
  if [[ $HAS_SRT -eq 1 ]]; then
    echo "  ffplay -fflags nobuffer -flags low_delay -probesize 32k -an -sync video srt://<IP>:${PORT}?mode=caller&latency=80"
  else
    echo "  ffplay -fflags nobuffer -flags low_delay -probesize 32k -an -sync video udp://<IP>:${PORT}"
  fi
done

fail=0; for p in "${pids[@]}"; do wait "$p" || fail=1; done
echo; echo "[i] files:"; find "$OUTROOT" -maxdepth 3 -type f -path "*/${STAMP}/out.avi" -print
exit $fail
