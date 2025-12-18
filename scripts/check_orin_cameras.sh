#!/usr/bin/env bash
set -euo pipefail

NUM_BUFFERS="${NUM_BUFFERS:-5}"
MEASURE_BUFFERS="${MEASURE_BUFFERS:-30}"
TIMEOUT_S="${TIMEOUT_S:-60}"
FPS="${FPS:-30}"
CONTENT_CHECK="${CONTENT_CHECK:-1}"
SAMPLE_BYTES="${SAMPLE_BYTES:-200000}"
KEEP_RAW="${KEEP_RAW:-0}"

# Optional: space-separated list, e.g.:
#   DEVICES="/dev/video0 /dev/video1 /dev/video2"
DEVICES="${DEVICES:-}"

# Optional: bash regex to filter by v4l2-ctl "Card type" (empty = no filter).
CARD_TYPE_REGEX="${CARD_TYPE_REGEX:-}"

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
echo "content_check=${CONTENT_CHECK} sample_bytes=${SAMPLE_BYTES} keep_raw=${KEEP_RAW}"
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

v4l2_pixfmt_to_gst_format() {
  local pixfmt="$1"
  case "${pixfmt}" in
    YUYV) echo "YUY2" ;;
    YUY2) echo "YUY2" ;;
    UYVY) echo "UYVY" ;;
    VYUY) echo "VYUY" ;;
    YVYU) echo "YVYU" ;;
    NV16) echo "NV16" ;;
    NV12) echo "NV12" ;;
    *) echo "${pixfmt}" ;;
  esac
}

for d in "${devices[@]}"; do
  [[ -e "${d}" ]] || continue
  echo "===== ${d} ====="

  card_type="$(v4l2-ctl -d "${d}" -D 2>/dev/null | awk -F': ' '/Card type/ {print $2; exit}')"
  bus_info="$(v4l2-ctl -d "${d}" -D 2>/dev/null | awk -F': ' '/Bus info/ {print $2; exit}')"

  if [[ -z "${card_type}" ]]; then
    echo "skip: card_type=unknown bus_info=${bus_info:-unknown}"
    echo ""
    continue
  fi

  echo "card_type=${card_type}"
  echo "bus_info=${bus_info}"

  if [[ -n "${CARD_TYPE_REGEX}" && ! "${card_type}" =~ ${CARD_TYPE_REGEX} ]]; then
    echo "skip: card_type does not match CARD_TYPE_REGEX=${CARD_TYPE_REGEX}"
    echo ""
    continue
  fi

  fmt="$(v4l2-ctl -d "${d}" --get-fmt-video 2>/dev/null || true)"
  if [[ -z "${fmt}" ]]; then
    echo "WARN: failed to read current format" >&2
    echo ""
    continue
  fi

  width="$(awk -F'[:/ ]+' '/Width\/Height/ {print $3; exit}' <<<"${fmt}")"
  height="$(awk -F'[:/ ]+' '/Width\/Height/ {print $4; exit}' <<<"${fmt}")"
  pixfmt="$(awk -F"'" '/Pixel Format/ {print $2; exit}' <<<"${fmt}")"

  if [[ -z "${width}" || -z "${height}" ]]; then
    echo "WARN: failed to parse width/height from v4l2-ctl output" >&2
    echo ""
    continue
  fi

  gst_format="$(v4l2_pixfmt_to_gst_format "${pixfmt}")"
  echo "fmt=${width}x${height} pixfmt=${pixfmt:-unknown} gst_format=${gst_format:-unknown}"

  echo "-- gst sanity (${NUM_BUFFERS} buffers) --"
  timeout "${TIMEOUT_S}s" \
    gst-launch-1.0 -q v4l2src device="${d}" num-buffers="${NUM_BUFFERS}" \
      ! "video/x-raw,format=${gst_format},width=${width},height=${height},framerate=${FPS}/1" \
      ! fakesink sync=false
  echo "OK"

  if (( CONTENT_CHECK > 0 )); then
    if command -v python3 >/dev/null 2>&1; then
      tmp_raw="/tmp/cr_cam_$(basename "${d}").raw"
      echo "-- content check (1 buffer -> ${tmp_raw}) --"
      timeout "${TIMEOUT_S}s" \
        gst-launch-1.0 -q v4l2src device="${d}" num-buffers=1 \
          ! "video/x-raw,format=${gst_format},width=${width},height=${height},framerate=${FPS}/1" \
          ! filesink location="${tmp_raw}"

      python3 - "${tmp_raw}" "${SAMPLE_BYTES}" <<'PY'
import math
import sys

path = sys.argv[1]
sample_bytes = int(sys.argv[2])

with open(path, "rb") as f:
  data = f.read(sample_bytes)

if not data:
  print("content_check: empty")
  raise SystemExit(0)

n = len(data)
mean = sum(data) / n
var = sum((b - mean) * (b - mean) for b in data) / n
std = math.sqrt(var)

flag = "OK"
if std < 0.5:
  flag = "FLAT (likely solid-color / frozen stream)"

print(f"content_check: sample_bytes={n} mean={mean:.3f} std={std:.3f} => {flag}")
PY

      if (( KEEP_RAW == 0 )); then
        rm -f "${tmp_raw}" || true
      else
        echo "kept_raw=${tmp_raw}"
      fi
    else
      echo "skip content check: python3 not found"
    fi
  fi

  if (( MEASURE_BUFFERS > 0 )); then
    echo "-- gst timing (${MEASURE_BUFFERS} buffers) --"
    { time timeout "${TIMEOUT_S}s" \
      gst-launch-1.0 -q v4l2src device="${d}" num-buffers="${MEASURE_BUFFERS}" \
        ! "video/x-raw,format=${gst_format},width=${width},height=${height},framerate=${FPS}/1" \
        ! fakesink sync=false; } 2>&1 | tail -n 1
  fi

  echo ""
done
