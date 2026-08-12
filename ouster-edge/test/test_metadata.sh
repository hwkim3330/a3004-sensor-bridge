#!/bin/sh
# Exercise ouster-metadata against a mock sensor HTTP API.
#
# The script only uses uclient-fetch and jsonfilter, which exist on OpenWrt and
# not on a build host, so this provides shims for both. What is being tested is
# the script's own logic: the firmware 2.x / 1.x endpoint fallback, field
# extraction, the defaults when a field is absent, and non-zero exit when the
# sensor is unreachable.
#
# Usage: sh test_metadata.sh

set -u
SCRIPT="${OUSTER_METADATA:-../files/ouster-metadata}"
TMP="$(mktemp -d)"
FAILED=0

cleanup() {
	[ -n "${SRV_PID:-}" ] && kill "$SRV_PID" 2>/dev/null
	rm -rf "$TMP"
}
trap cleanup EXIT

check() {
	if [ "$2" = "$3" ]; then
		echo "  PASS  $1: $2"
	else
		echo "  FAIL  $1: got '$2' want '$3'"
		FAILED=1
	fi
}

# --- shims -----------------------------------------------------------------
mkdir -p "$TMP/bin"

cat > "$TMP/bin/uclient-fetch" <<'EOF'
#!/bin/sh
# uclient-fetch -q -T 5 -O <out> <url>
OUT=""
while [ $# -gt 0 ]; do
	case "$1" in
		-O) OUT="$2"; shift 2;;
		-T) shift 2;;
		-q) shift;;
		*) URL="$1"; shift;;
	esac
done
exec curl -sfL --max-time 5 -o "$OUT" "$URL"
EOF

cat > "$TMP/bin/jsonfilter" <<'EOF'
#!/bin/sh
# jsonfilter -i <file> -e '$.a.b'  — enough of the expression syntax for this use
while [ $# -gt 0 ]; do
	case "$1" in
		-i) FILE="$2"; shift 2;;
		-e) EXPR="$2"; shift 2;;
		*) shift;;
	esac
done
python3 - "$FILE" "$EXPR" <<'PY'
import json, sys
doc = json.load(open(sys.argv[1]))
path = sys.argv[2].lstrip('$').lstrip('.')
node = doc
for part in [p for p in path.split('.') if p]:
    if not isinstance(node, dict) or part not in node:
        sys.exit(1)
    node = node[part]
print(node)
PY
EOF
chmod +x "$TMP/bin/uclient-fetch" "$TMP/bin/jsonfilter"
PATH="$TMP/bin:$PATH"
export PATH

# --- mock sensor -----------------------------------------------------------
mkdir -p "$TMP/www/api/v1/sensor/metadata"

start_server() {
	( cd "$TMP/www" && python3 -m http.server 18080 >/dev/null 2>&1 ) &
	SRV_PID=$!
	i=0
	while [ $i -lt 40 ]; do
		curl -sf -o /dev/null "http://127.0.0.1:18080/" && return 0
		i=$((i + 1))
		sleep 0.1
	done
	echo "mock server did not start" >&2
	exit 2
}

echo "ouster-metadata verification"

# Firmware 2.x: the whole metadata blob, data_format nested inside.
cat > "$TMP/www/api/v1/sensor/metadata/index.html" <<'EOF'
{
  "sensor_info": {"prod_line": "OS-1-64", "build_rev": "v2.5.2"},
  "data_format": {
    "pixels_per_column": 64,
    "columns_per_packet": 16,
    "columns_per_frame": 1024,
    "udp_profile_lidar": "RNG19_RFL8_SIG16_NIR16",
    "pixel_shift_by_row": [0, 0, 0, 0]
  }
}
EOF
start_server

echo "--- firmware 2.x endpoint ---"
OUT="$(sh "$SCRIPT" 127.0.0.1:18080 2>/dev/null)"
eval "$OUT"
check "channels"   "${OUSTER_CHANNELS:-}"   "64"
check "columns"    "${OUSTER_COLUMNS:-}"    "16"
check "scan width" "${OUSTER_SCAN_WIDTH:-}" "1024"
check "profile"    "${OUSTER_PROFILE:-}"    "RNG19_RFL8_SIG16_NIR16"

echo "--- 2048x10 mode is picked up, not assumed ---"
sed -i 's/"columns_per_frame": 1024/"columns_per_frame": 2048/' \
	"$TMP/www/api/v1/sensor/metadata/index.html"
unset OUSTER_SCAN_WIDTH
eval "$(sh "$SCRIPT" 127.0.0.1:18080 2>/dev/null)"
check "scan width" "${OUSTER_SCAN_WIDTH:-}" "2048"

echo "--- firmware 1.x fallback (no /metadata, only /metadata/lidar_data_format) ---"
rm -f "$TMP/www/api/v1/sensor/metadata/index.html"
cat > "$TMP/www/api/v1/sensor/metadata/lidar_data_format" <<'EOF'
{
  "pixels_per_column": 128,
  "columns_per_packet": 16,
  "columns_per_frame": 512,
  "udp_profile_lidar": "LEGACY"
}
EOF
unset OUSTER_CHANNELS OUSTER_SCAN_WIDTH OUSTER_PROFILE
eval "$(sh "$SCRIPT" 127.0.0.1:18080 2>/dev/null)"
check "channels"   "${OUSTER_CHANNELS:-}"   "128"
check "scan width" "${OUSTER_SCAN_WIDTH:-}" "512"
check "profile"    "${OUSTER_PROFILE:-}"    "LEGACY"

echo "--- missing optional fields fall back to defaults ---"
cat > "$TMP/www/api/v1/sensor/metadata/lidar_data_format" <<'EOF'
{ "pixels_per_column": 32 }
EOF
unset OUSTER_CHANNELS OUSTER_COLUMNS OUSTER_SCAN_WIDTH OUSTER_PROFILE
eval "$(sh "$SCRIPT" 127.0.0.1:18080 2>/dev/null)"
check "channels" "${OUSTER_CHANNELS:-}"   "32"
check "columns"  "${OUSTER_COLUMNS:-}"    "16"
check "width"    "${OUSTER_SCAN_WIDTH:-}" "1024"
check "profile"  "${OUSTER_PROFILE:-}"    "LEGACY"

echo "--- unreachable sensor exits non-zero and prints nothing on stdout ---"
OUT="$(sh "$SCRIPT" 127.0.0.1:18099 2>/dev/null)"; RC=$?
check "exit code" "$RC" "1"
check "stdout empty" "$(printf '%s' "$OUT" | wc -c | tr -d ' ')" "0"

echo "--- no argument is a usage error ---"
sh "$SCRIPT" >/dev/null 2>&1; RC=$?
check "exit code" "$RC" "2"

echo
if [ "$FAILED" = "0" ]; then
	echo "all checks passed"
else
	echo "FAILED"
fi
exit "$FAILED"
