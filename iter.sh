#!/bin/bash
set -u

REMOTE_USER="agravelot"
REMOTE_HOST="voron2.agravelot.eu"
API="http://${REMOTE_HOST}:7125"
API_KEY="1d2407061bfc461a8fe49a1e05466236"
CURL="curl -sf -H X-Api-Key:${API_KEY}"
WAIT_MAX=60

wait_klipper() {
  for i in $(seq 1 $WAIT_MAX); do
    st=$($CURL "$API/printer/info" 2>/dev/null |
      python3 -c "import sys,json; print(json.load(sys.stdin).get('result',{}).get('state',''))" 2>/dev/null)
    [ "$st" = "ready" ] && {
      echo "  Klipper ready (${i}s)"
      return 0
    }
    [ "$st" = "error" ] && {
      echo "  ERROR state - pulling log..."
      mkdir -p logs 2>/dev/null
      scp -o StrictHostKeyChecking=no "${REMOTE_USER}@${REMOTE_HOST}:~/printer_data/logs/klippy.log" "logs/klippy.log" 2>/dev/null
      echo "  <<< klippy.log tail >>>"
      grep -v "^Stats\|^Sent\|^Receive:" logs/klippy.log | tail -20 || true
      echo ""
      return 2
    }
    sleep 1
  done
  echo "  ERROR: not ready after ${WAIT_MAX}s"
  return 1
}

wait_idle() {
  for i in $(seq 1 $WAIT_MAX); do
    st=$($CURL "$API/printer/objects/query?print_stats" 2>/dev/null |
      python3 -c "import sys,json; print(json.load(sys.stdin)['result']['status']['print_stats'].get('state',''))" 2>/dev/null)
    [ "$st" = "standby" ] || [ "$st" = "complete" ] && {
      echo "  Idle (${i}s)"
      return 0
    }
    [ "$st" = "paused" ] && continue
    sleep 1
  done
}

send_gcode() {
  $CURL -X POST "$API/printer/gcode/script" \
    -H "Content-Type: application/json" \
    -d "{\"script\": \"$1\"}" >/dev/null 2>&1
}

echo "  Push..."
# Sync entire kalico repo to printer's klipper directory
rsync -e "ssh -o StrictHostKeyChecking=no" --update \
  ~/lab/kalico/ \
  "${REMOTE_USER}@${REMOTE_HOST}:~/klipper/" || exit 1

# C code - use rsync for efficient incremental file transfer
echo "  Checking for C changes..."
if git diff --quiet HEAD src/*.c 2>/dev/null; then
  echo "  No C changes detected"
else
  echo "  C code changed - using rsync to update printer build dir..."

  PRINTER_KLIPPER="~/klipper"
  LOCAL_SRC="$(pwd)/src" # ~/lab/kalico/src on home machine
  REMOTE_SRC="$PRINTER_KLIPPER/src"

  echo "  Syncing changed files from $LOCAL_SRC to $REMOTE_SRC..."
  # Use rsync with --update flag - only copies newer or missing files
  # This is much faster than scp for incremental updates
  rsync -e "ssh -o StrictHostKeyChecking=no" \
    --update \
    -r \
    "$LOCAL_SRC/" \
    "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_SRC}/"

  # Small delay to ensure filesystem sync
  sleep 0.5

  # Determine board type - use octopus for Voron2
  CONFIG_FILE="../voron2-config/scripts/octopus.config"

  PRINTER_CFG="~/printer_data/config/scripts/$(basename $CONFIG_FILE .config).config"

  # Always sync config and run olddefconfig for consistent builds
  scp "$CONFIG_FILE" "${REMOTE_USER}@${REMOTE_HOST}:${PRINTER_CFG}" || exit 1
  ssh "${REMOTE_USER}@${REMOTE_HOST}" \
    "cd ~/klipper && env KCONFIG_CONFIG=${PRINTER_CFG} make olddefconfig" || exit 1

  # Run update script to build & flash
  ssh "${REMOTE_USER}@${REMOTE_HOST}" \
    "bash ~/printer_data/config/scripts/update-klipper.sh " || exit 1
  if ! wait_klipper; then
    echo "  Post-flash recovery: FIRMWARE_RESTART"
    $CURL -X POST "$API/printer/gcode/script" \
      -H "Content-Type: application/json" \
      -d '{"script":"FIRMWARE_RESTART"}' >/dev/null 2>&1
    wait_klipper || exit 1
  fi
fi

echo "  Restart..."
# Rotate log before restart so we only see current test output
ssh -o StrictHostKeyChecking=no "${REMOTE_USER}@${REMOTE_HOST}" \
  "> ~/printer_data/logs/klippy.log" 2>/dev/null || true
$CURL -X POST "$API/printer/gcode/script" \
  -H "Content-Type: application/json" \
  -d '{"script":"FIRMWARE_RESTART"}' >/dev/null 2>&1
wait_klipper || exit 1
echo "  _CG28..."
send_gcode "_CG28"
sleep 1

wait_idle

# Pull logs
mkdir -p logs 2>/dev/null
scp "${REMOTE_USER}@${REMOTE_HOST}:~/printer_data/logs/klippy.log" "logs/klippy.log" >/dev/null 2>&1 || true

echo ""
echo "  <<< corexy_home activity >>>"
grep -v "^Stats\|^Sent\|^Receive:" logs/klippy.log | grep -i "corexy_home\|Move out of range\|Error" | tail -20 || true
echo ""

if grep -q "Move out of range" logs/klippy.log 2>/dev/null; then
  echo "  *** OUT OF RANGE ***"
  exit 1
fi

if grep -qi "error\|fatal" logs/klippy.log 2>/dev/null; then
  echo "  *** ERROR ***"
  exit 1
fi

echo "  all good"
