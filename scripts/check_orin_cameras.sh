#!/usr/bin/env bash
set -euo pipefail

NUM_BUFFERS="${NUM_BUFFERS:-5}"
MEASURE_BUFFERS="${MEASURE_BUFFERS:-30}"
TIMEOUT_S="${TIMEOUT_S:-60}"
FPS="${FPS:-30}"

# Optional: space-separated list, e.g.:
#   DEVICES="/dev/video0 /dev/video1 /dev/video2"
DEVICES="${DEVICES:-}"

if ! command -v v4l2-ctl >/dev/null 2>&1; then
  echo "ERROR: v4l2-ctl not found" >&2
  exit 1
fi

if ! command -v gst-launch-1.0 >/dev/null 2>&1; then
  echo "ERROR: gst-launch-1.0 not found" >&2
  exit 1
fi

if [[ -n "${DEVICES}" ]]; then
  read -r -a devices <<<"${DEVICES}"
else
  mapfile -t devices < <(ls -1 /dev/video* 2>/dev/null | sort -V || true)
fi

if (( ${#devices[@]} == 0 )); then
  echo "ERROR: no /dev/video* devices found" >&2
  exit 1
fi

echo "num_buffers=${NUM_BUFFERS} measure_buffers=${MEASURE_BUFFERS} timeout_s=${TIMEOUT_S} fps=${FPS}"
echo "devices: ${devices[*]}"
echo ""

if command -v media-ctl >/dev/null 2>&1 && [[ -e /dev/media0 ]]; then
  echo "==== media-ctl mapping (/dev/media0) ===="
  media-ctl -p -d /dev/media0 2>/dev/null | awk '
    /^- entity/ { entity=$0 }
    /type V4L2 subdev subtype Sensor/ { sensor=1 }
    /device node name \/dev\/video/ && sensor==1 { print entity; print "    " $0; sensor=0 }
  ' || true
  echo ""
fi

TIMEFORMAT="sec=%R"

for d in "${devices[@]}"; do
  [[ -e "${d}" ]] || continue
  echo "===== ${d} ====="

  card_type="$(v4l2-ctl -d "${d}" -D 2>/dev/null | awk -F': ' '/Card type/ {print $2; exit}')"
  bus_info="$(v4l2-ctl -d "${d}" -D 2>/dev/null | awk -F': ' '/Bus info/ {print $2; exit}')"

  if [[ "${card_type}" != *"sgx-yuv-gmsl2"* ]]; then
    echo "skip: card_type=${card_type:-unknown} bus_info=${bus_info:-unknown}"
    echo ""
    continue
  fi

  echo "card_type=${card_type}"
  echo "bus_info=${bus_info}"

  fmt="$(v4l2-ctl -d "${d}" --get-fmt-video 2>/dev/null || true)"
  if [[ -z "${fmt}" ]]; then
    echo "WARN: failed to read current format" >&2
    echo ""
    continue
  fi

  width="$(awk -F'[:/ ]+' '/Width\/Height/ {print $3; exit}' <<<"${fmt}")"
  height="$(awk -F'[:/ ]+' '/Width\/Height/ {print $4; exit}' <<<"${fmt}")"

  if [[ -z "${width}" || -z "${height}" ]]; then
    echo "WARN: failed to parse width/height from v4l2-ctl output" >&2
    echo ""
    continue
  fi

  echo "fmt=${width}x${height}"

  echo "-- gst sanity (${NUM_BUFFERS} buffers) --"
  timeout "${TIMEOUT_S}s" \
    gst-launch-1.0 -q v4l2src device="${d}" num-buffers="${NUM_BUFFERS}" \
      ! "video/x-raw,format=UYVY,width=${width},height=${height},framerate=${FPS}/1" \
      ! fakesink sync=false
  echo "OK"

  if (( MEASURE_BUFFERS > 0 )); then
    echo "-- gst timing (${MEASURE_BUFFERS} buffers) --"
    { time timeout "${TIMEOUT_S}s" \
      gst-launch-1.0 -q v4l2src device="${d}" num-buffers="${MEASURE_BUFFERS}" \
        ! "video/x-raw,format=UYVY,width=${width},height=${height},framerate=${FPS}/1" \
        ! fakesink sync=false; } 2>&1 | tail -n 1
  fi

  echo ""
done
