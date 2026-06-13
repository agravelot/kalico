#!/bin/bash
set -u

FORCE=false
NO_FLASH=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --force|-f) FORCE=true; shift ;;
    --no-flash|-n) NO_FLASH=true; shift ;;
    *) echo "Usage: $0 [--force|-f] [--no-flash|-n]"; exit 1 ;;
  esac
done

REMOTE_USER="agravelot"
REMOTE_HOST="voron2.agravelot.eu"
API="http://${REMOTE_HOST}:7125"
API_KEY="1d2407061bfc461a8fe49a1e05466236"
CURL="curl -sf -H X-Api-Key:${API_KEY}"
WAIT_MAX=60

# State file: tracks the last commit that was successfully pushed to the
# printer. Used to decide whether to (a) build & flash firmware, (b) restart
# Klippy, or (c) do nothing.
STATE_DIR="$(dirname "$(readlink -f "$0")")/.iter-state"
LAST_PUSH_FILE="$STATE_DIR/last-push"

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

record_last_push() {
  mkdir -p "$STATE_DIR" 2>/dev/null
  git rev-parse HEAD > "$LAST_PUSH_FILE"
}

last_pushed_commit() {
  [ -f "$LAST_PUSH_FILE" ] || return 1
  cat "$LAST_PUSH_FILE"
  return 0
}

# Decide what needs to happen this run.
HEAD=$(git rev-parse HEAD)
LAST=$(last_pushed_commit 2>/dev/null || echo "")
echo "  HEAD:  ${HEAD:0:12}"
[ -n "$LAST" ] && echo "  last:  ${LAST:0:12}"

# What changed since the last push?
NEED_RSYNC=false
NEED_FLASH=false
NEED_RESTART=false

if $FORCE; then
  echo "  Force: rsync + flash + restart"
  NEED_RSYNC=true
  NEED_FLASH=true
  NEED_RESTART=true
elif [ -z "$LAST" ] || [ "$HEAD" != "$LAST" ]; then
  NEED_RSYNC=true
  NEED_RESTART=true
  if [ -z "$LAST" ]; then
    echo "  No last-push state - first run, will rsync + restart + flash"
    NEED_FLASH=true
  else
    # C changed since last push?
    if ! git diff --quiet "$LAST" HEAD -- src/ 2>/dev/null; then
      NEED_FLASH=true
      echo "  src/ changed since ${LAST:0:8} - will rebuild + flash"
    else
      echo "  src/ unchanged since ${LAST:0:8} - skipping C build"
    fi
    # Any tracked file changed?
    if git diff --quiet "$LAST" HEAD 2>/dev/null; then
      # HEAD changed but no diff (e.g. amended, rebased) - restart
      # anyway to make sure printer picks up the new code
      echo "  HEAD moved (no tracked diff) - will restart Klippy"
    else
      CHANGED=$(git diff --name-only "$LAST" HEAD 2>/dev/null | wc -l)
      echo "  $CHANGED file(s) changed - will rsync + restart"
    fi
  fi
else
  echo "  No changes since last push - nothing to do"
fi

# Uncommitted local changes? Always rsync + restart so the printer
# picks up the working tree state, even if HEAD already matches.
UNCOMMITTED_C=false
UNCOMMITTED_OTHER=false
if ! git diff --quiet HEAD -- src/ 2>/dev/null; then
  UNCOMMITTED_C=true
fi
if ! git diff --quiet HEAD 2>/dev/null; then
  UNCOMMITTED_OTHER=true
fi

if $UNCOMMITTED_C || $UNCOMMITTED_OTHER; then
  NEED_RSYNC=true
  NEED_RESTART=true
  if $UNCOMMITTED_C; then
    NEED_FLASH=true
    echo "  Uncommitted C changes - will rebuild + flash"
  else
    echo "  Uncommitted non-C changes - will rsync + restart"
  fi
fi

if $NO_FLASH; then
  echo "  --no-flash: skipping firmware flash"
  NEED_FLASH=false
fi

if ! $NEED_RSYNC && ! $NEED_FLASH && ! $NEED_RESTART; then
  echo "  all good (no changes)"
  exit 0
fi

echo "  Push..."
# Sync entire kalico repo to printer's klipper directory
RSYNC_OPTS="-rlpt"
rsync -e "ssh -o StrictHostKeyChecking=no" $RSYNC_OPTS \
  --exclude='.git/' --exclude='logs/' --exclude='out/' --exclude='__pycache__/' \
  --exclude='.iter-state/' \
  ~/lab/kalico/ \
  "${REMOTE_USER}@${REMOTE_HOST}:~/klipper/" || exit 1

if $NEED_FLASH; then
  echo "  C build + flash..."

  PRINTER_KLIPPER="~/klipper"
  REMOTE_SRC="$PRINTER_KLIPPER/src"

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
  # The post-flash Klippy start counts as the restart.
  NEED_RESTART=false
fi

if $NEED_RESTART; then
  echo "  Restart Klippy..."
  $CURL -X POST "$API/server/logs/rollover" \
    -H "Content-Type: application/json" \
    -d '{"application": "klipper"}' >/dev/null 2>&1 || true
  $CURL -X POST "$API/printer/gcode/script" \
    -H "Content-Type: application/json" \
    -d '{"script":"FIRMWARE_RESTART"}' >/dev/null 2>&1
  wait_klipper || exit 1
fi

record_last_push

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

if grep -qi "config error\|traceback\|unhandled exception\|internal error\|mcu error\|fatal" logs/klippy.log 2>/dev/null; then
  echo "  *** ERROR ***"
  exit 1
fi

echo "  all good"
