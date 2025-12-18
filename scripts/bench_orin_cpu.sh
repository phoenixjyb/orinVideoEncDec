#!/usr/bin/env bash
set -euo pipefail

MAX_CAMS="${MAX_CAMS:-5}"
DURATION_S="${DURATION_S:-20}"

INPUT_WIDTH="${INPUT_WIDTH:-1920}"
INPUT_HEIGHT="${INPUT_HEIGHT:-1536}"
INPUT_FORMAT="${INPUT_FORMAT:-UYVY}"
OUTPUT_WIDTH="${OUTPUT_WIDTH:-960}"
OUTPUT_HEIGHT="${OUTPUT_HEIGHT:-768}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-4000000}"

usage() {
  cat <<'EOF'
bench_orin_cpu.sh

Benchmark CPU usage scaling for cr_h265_publisher as cameras increase.

Expected usage (on Orin):
  source /opt/ros/humble/setup.bash
  source /home/nvidia/wkp/install/setup.bash
  bash scripts/bench_orin_cpu.sh

Env overrides:
  MAX_CAMS=5
  DURATION_S=20
  INPUT_WIDTH=1920 INPUT_HEIGHT=1536
  INPUT_FORMAT=UYVY
  OUTPUT_WIDTH=960 OUTPUT_HEIGHT=768
  FPS=30
  BITRATE=4000000

Optional:
  DEV_INDICES="0 1 2 3 4"  # defaults to 0..4
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 not found in PATH (did you source /opt/ros/humble/setup.bash?)" >&2
  exit 1
fi

NODE_PREFIX="$(ros2 pkg prefix cr_h265_publisher)"
NODE_BIN="${NODE_PREFIX}/lib/cr_h265_publisher/cr_h265_publisher_node"

if [[ ! -x "${NODE_BIN}" ]]; then
  echo "ERROR: node binary not found/executable: ${NODE_BIN}" >&2
  exit 1
fi

CLK_TCK="$(getconf CLK_TCK)"

read_proc_jiffies() {
  local pid="$1"
  awk '{print $14+$15}' "/proc/${pid}/stat"
}

read_sys_cpu() {
  awk '/^cpu / {print $2,$3,$4,$5,$6,$7,$8,$9}' /proc/stat
}

cpu_usage_pct() {
  local idle0="$1"
  local total0="$2"
  local idle1="$3"
  local total1="$4"
  awk -v idle0="$idle0" -v total0="$total0" -v idle1="$idle1" -v total1="$total1" 'BEGIN{
    d_idle = idle1 - idle0;
    d_total = total1 - total0;
    if (d_total <= 0) { print "0.00"; exit }
    printf "%.2f", (1.0 - (d_idle / d_total)) * 100.0;
  }'
}

proc_usage_pct() {
  local dj="$1"
  local dt_ns="$2"
  awk -v dj="$dj" -v clk="$CLK_TCK" -v dt_ns="$dt_ns" 'BEGIN{
    if (dt_ns <= 0) { print "0.00"; exit }
    dt = dt_ns / 1000000000.0;
    printf "%.2f", (dj/clk) / dt * 100.0;
  }'
}

join_by() {
  local IFS="$1"
  shift
  echo "$*"
}

DEV_INDICES="${DEV_INDICES:-0 1 2 3 4}"
mapfile -t ALL_DEVICES < <(for i in ${DEV_INDICES}; do echo "/dev/video${i}"; done)

if (( MAX_CAMS < 1 )); then
  echo "ERROR: MAX_CAMS must be >= 1" >&2
  exit 1
fi

echo "node_bin=${NODE_BIN}"
echo "duration_s=${DURATION_S}"
echo "input=${INPUT_WIDTH}x${INPUT_HEIGHT}@${FPS} format=${INPUT_FORMAT}"
echo "output=${OUTPUT_WIDTH}x${OUTPUT_HEIGHT} bitrate=${BITRATE}"
echo ""

printf "%-6s %-12s %-12s %-12s\n" "cams" "proc_cpu(%)" "sys_cpu(%)" "pid"

for (( n = 1; n <= MAX_CAMS; n++ )); do
  devices=("${ALL_DEVICES[@]:0:n}")
  for d in "${devices[@]}"; do
    if [[ ! -e "${d}" ]]; then
      echo "WARN: missing device ${d}; stopping at cams=$((n-1))" >&2
      exit 0
    fi
  done

  topics=()
  frame_ids=()
  for (( i = 0; i < n; i++ )); do
    topics+=("/cr/camera/h265/bench${i}")
    frame_ids+=("cam${i}")
  done

  devices_arg="[$(join_by , "${devices[@]}")]"
  topics_arg="[$(join_by , "${topics[@]}")]"
  frame_ids_arg="[$(join_by , "${frame_ids[@]}")]"

  set +e
  "${NODE_BIN}" --ros-args \
    -p "devices:=${devices_arg}" \
    -p "topics:=${topics_arg}" \
    -p "frame_ids:=${frame_ids_arg}" \
    -p "input_format:=${INPUT_FORMAT}" \
    -p "input_width:=${INPUT_WIDTH}" \
    -p "input_height:=${INPUT_HEIGHT}" \
    -p "output_width:=${OUTPUT_WIDTH}" \
    -p "output_height:=${OUTPUT_HEIGHT}" \
    -p "fps:=${FPS}" \
    -p "bitrate:=${BITRATE}" \
    >/tmp/cr_h265_publisher_bench_${n}.log 2>&1 &
  pid=$!
  set -e

  sleep 2
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "ERROR: publisher exited early for cams=${n} (see /tmp/cr_h265_publisher_bench_${n}.log)" >&2
    exit 1
  fi

  sys0="$(read_sys_cpu)"
  idle0="$(awk '{print $4+$5}' <<<"$sys0")"
  total0="$(awk '{print $1+$2+$3+$4+$5+$6+$7+$8}' <<<"$sys0")"

  t0_ns="$(date +%s%N)"
  j0="$(read_proc_jiffies "$pid")"

  sleep "${DURATION_S}"

  t1_ns="$(date +%s%N)"
  j1="$(read_proc_jiffies "$pid")"

  sys1="$(read_sys_cpu)"
  idle1="$(awk '{print $4+$5}' <<<"$sys1")"
  total1="$(awk '{print $1+$2+$3+$4+$5+$6+$7+$8}' <<<"$sys1")"

  dt_ns="$((t1_ns - t0_ns))"
  dj="$((j1 - j0))"

  proc_pct="$(proc_usage_pct "$dj" "$dt_ns")"
  sys_pct="$(cpu_usage_pct "$idle0" "$total0" "$idle1" "$total1")"

  printf "%-6s %-12s %-12s %-12s\n" "${n}" "${proc_pct}" "${sys_pct}" "${pid}"

  kill "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
done
