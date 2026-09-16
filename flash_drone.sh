#!/usr/bin/env bash
#
# Build and flash one drone.
#
#   ./flash_drone.sh            # prompts for drone id, then port
#   ./flash_drone.sh 22         # drone 22, prompts for port
#   ./flash_drone.sh 22 /dev/tty.usbmodem2101
#
# sdkconfig is tracked in git, so it is pinned to this drone only for the
# build and restored afterwards -- on success, on failure and on Ctrl-C.

set -euo pipefail

PORT_DEFAULT="/dev/tty.usbmodem2101"
ID_MAX=30                  # main/Kconfig.projbuild: config DRONE_ID, range 0 30
IP_PREFIX="192.168.1"      # host IP last octet = IP_BASE + drone id
IP_BASE=100

die()  { printf '\nERROR: %s\n' "$*" >&2; exit 1; }
step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

# Resolve the project from the script's own location, so cwd does not matter.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && git rev-parse --show-toplevel 2>/dev/null)" \
  || die "not inside a git work tree"
cd "$PROJECT_DIR"
SDKCONFIG="$PROJECT_DIR/sdkconfig"
[ -f "$SDKCONFIG" ] || die "no sdkconfig in $PROJECT_DIR"
git ls-files --error-unmatch sdkconfig >/dev/null 2>&1 \
  || die "sdkconfig is not tracked by git -- refusing to edit a file we cannot restore"

# ---------------------------------------------------------------- 1. ESP-IDF
step "ESP-IDF environment"

# Search order: an explicit/already-exported IDF_PATH, then conventional
# locations. No machine-specific path is baked in -- cloning esp-idf next to
# this repo (see tools/idf-pin.env) is enough.
find_idf() {
    for c in "${IDF_PATH:-}" "$PROJECT_DIR/esp-idf" "$PROJECT_DIR/../esp-idf" \
             "$HOME/esp/esp-idf" "$HOME/esp-idf"; do
        if [ -n "$c" ] && [ -f "$c/export.sh" ]; then
            ( cd "$c" && pwd )                # normalise ../ away
            return 0
        fi
    done
    return 1
}

if command -v idf.py >/dev/null 2>&1; then
    echo "already sourced: ${IDF_PATH:-?}"
else
    IDF_PATH="$(find_idf)" || die "ESP-IDF not found. Clone it next to this repo:
  git clone https://github.com/espressif/esp-idf.git \"$PROJECT_DIR/../esp-idf\"
then follow tools/idf-pin.env (checkout the pin, ./install.sh esp32s3)."
    export IDF_PATH
    echo "sourcing $IDF_PATH/export.sh ..."
    set +eu                                   # export.sh is not -e/-u clean
    . "$IDF_PATH/export.sh" >/tmp/idf_export.$$.log 2>&1
    set -eu
    command -v idf.py >/dev/null 2>&1 || {
        cat /tmp/idf_export.$$.log >&2
        die "export.sh did not put idf.py on PATH"
    }
    rm -f /tmp/idf_export.$$.log
fi
echo "using $(command -v idf.py) -- $(idf.py --version 2>/dev/null)"

# Warn, but never refuse, when the found ESP-IDF is not the pinned one.
PIN="$PROJECT_DIR/tools/idf-pin.env"
if [ -f "$PIN" ] && [ -n "${IDF_PATH:-}" ]; then
    want="$(sed -n 's/^IDF_CHECKOUT_REF=//p' "$PIN")"
    have="$(git -C "$IDF_PATH" rev-parse HEAD 2>/dev/null || echo unknown)"
    if [ -n "$want" ] && [ "$want" != "$have" ]; then
        echo "WARNING: ESP-IDF at $IDF_PATH is $have"
        echo "         pinned   $want ($(sed -n 's/^IDF_EXPECTED_DESCRIBE=//p' "$PIN"))"
        echo "         firmware may differ from the team build; see tools/idf-pin.env"
    fi
fi

# -------------------------------------------------- 2 + 6. sdkconfig custody
# Install the trap BEFORE the first mutation so every exit path restores.
BACKUP="$(mktemp -t sdkconfig.before)"
cp "$SDKCONFIG" "$BACKUP"
WAS_DIRTY=0
git diff --quiet -- sdkconfig || WAS_DIRTY=1
MUTATED=0                      # 1 once we have actually touched sdkconfig

cleanup() {
    rc=$?
    set +e                     # a failed write must never abort the restore
    trap '' PIPE HUP INT TERM  # nor may a signal kill us mid-restore
    trap - EXIT

    # Restore BEFORE any cosmetic output. If stdout is a closed pipe
    # (./flash_drone.sh | head) or a hung-up pty, printing first would abort
    # the handler under errexit and leave sdkconfig pinned to the wrong drone.
    msg=""
    if [ "$MUTATED" -eq 1 ]; then
        if git -C "$PROJECT_DIR" restore sdkconfig 2>/dev/null \
        || git -C "$PROJECT_DIR" checkout -- sdkconfig 2>/dev/null; then
            msg="sdkconfig restored to HEAD"
            if [ "$WAS_DIRTY" -eq 1 ]; then
                msg="$msg
NOTE: your pre-run sdkconfig edits are saved at $BACKUP"
            else
                rm -f "$BACKUP"
            fi
        else
            msg="COULD NOT RESTORE -- put it back by hand from: $BACKUP"
            [ "$rc" -eq 0 ] && rc=1
        fi
    else
        rm -f "$BACKUP"        # never touched it: nothing to restore or keep
    fi

    [ -n "$msg" ] && printf '\n\033[1;36m==> Restoring sdkconfig\033[0m\n%s\n' "$msg"
    if [ "$rc" -eq 0 ]; then printf '\033[1;32mDONE\033[0m\n'
    else                       printf '\033[1;31mFAILED (exit %s)\033[0m\n' "$rc"
    fi
    exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

step "Resetting sdkconfig"
if [ "$WAS_DIRTY" -eq 1 ]; then
    echo "WARNING: sdkconfig has uncommitted changes; they will be discarded."
    echo "         a copy is kept at $BACKUP"
    if [ -t 0 ]; then
        read -r -p "continue? [y/N] " a
        case "$a" in y|Y|yes|YES) ;; *) die "aborted by user";; esac
    fi
fi
MUTATED=1                      # from here on, cleanup() owns restoring the file
git restore sdkconfig
echo "sdkconfig reset to HEAD"

# ------------------------------------------------------------- 3. Drone ID
step "Drone configuration"
DRONE_ID="${1:-}"
if [ -z "$DRONE_ID" ]; then
    read -r -p "Drone ID (0-$ID_MAX): " DRONE_ID || die "no input: pass the drone id as an argument"
fi
case "$DRONE_ID" in ''|*[!0-9]*) die "drone id must be a whole number 0-$ID_MAX";; esac
[ "${#DRONE_ID}" -le 2 ] || die "drone id must be 0-$ID_MAX"   # also blocks $(( )) overflow
DRONE_ID=$((10#$DRONE_ID))                                      # 08 -> 8, not octal
[ "$DRONE_ID" -le "$ID_MAX" ] || die "drone id $DRONE_ID is outside the Kconfig range 0-$ID_MAX"
HOST_IP="$IP_PREFIX.$((IP_BASE + DRONE_ID))"

# awk, not sed -i: no BSD/GNU incompatibility, exact prefix match, appends if absent.
TMP="$(mktemp -t sdkconfig.new)"
awk -v id="$DRONE_ID" -v ip="$HOST_IP" '
  index($0, "CONFIG_DRONE_ID=")                  == 1 { print "CONFIG_DRONE_ID=" id;            si=1; next }
  index($0, "# CONFIG_DRONE_ID is not set")      == 1 { print "CONFIG_DRONE_ID=" id;            si=1; next }
  index($0, "CONFIG_HOST_IPV4_ADDR=")            == 1 { print "CONFIG_HOST_IPV4_ADDR=\"" ip "\""; sp=1; next }
  index($0, "# CONFIG_HOST_IPV4_ADDR is not set")== 1 { print "CONFIG_HOST_IPV4_ADDR=\"" ip "\""; sp=1; next }
  { print }
  END { if (!si) print "CONFIG_DRONE_ID=" id
        if (!sp) print "CONFIG_HOST_IPV4_ADDR=\"" ip "\"" }
' "$SDKCONFIG" > "$TMP"
cat "$TMP" > "$SDKCONFIG"      # keep the original inode/permissions
rm -f "$TMP"

# A substitution that matched nothing exits 0, so verify before burning a build on it.
grep -Fxq "CONFIG_DRONE_ID=$DRONE_ID"            "$SDKCONFIG" || die "failed to set CONFIG_DRONE_ID"
grep -Fxq "CONFIG_HOST_IPV4_ADDR=\"$HOST_IP\""   "$SDKCONFIG" || die "failed to set CONFIG_HOST_IPV4_ADDR"
[ "$(grep -c '^CONFIG_DRONE_ID=' "$SDKCONFIG")"       -eq 1 ] || die "duplicate CONFIG_DRONE_ID lines"
[ "$(grep -c '^CONFIG_HOST_IPV4_ADDR=' "$SDKCONFIG")" -eq 1 ] || die "duplicate CONFIG_HOST_IPV4_ADDR lines"
echo "drone $DRONE_ID  ->  host $HOST_IP"

# ---------------------------------------------------------------- 4. Build
step "Building drone $DRONE_ID"
idf.py build

# kconfgen rewrites sdkconfig during the build and silently resets out-of-range
# values, so trust the generated header, not what we wrote.
HDR="$PROJECT_DIR/build/config/sdkconfig.h"
grep -Fxq "#define CONFIG_DRONE_ID $DRONE_ID"              "$HDR" \
    || die "built binary is NOT drone $DRONE_ID ($(grep -m1 'define CONFIG_DRONE_ID ' "$HDR")) -- not flashing"
grep -Fxq "#define CONFIG_HOST_IPV4_ADDR \"$HOST_IP\""     "$HDR" \
    || die "built binary has the wrong host IP -- not flashing"
echo "verified: firmware is drone $DRONE_ID, host $HOST_IP"

# ---------------------------------------------------------------- 5. Flash
step "Flashing drone $DRONE_ID"
PORT="${2:-}"
while :; do
    if [ -z "$PORT" ]; then
        read -r -p "Port for drone $DRONE_ID [$PORT_DEFAULT]: " PORT || PORT=""
        PORT="${PORT:-$PORT_DEFAULT}"
    fi
    [ -e "$PORT" ] && break
    echo "port not found: $PORT"
    FOUND="$(ls /dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.wchusbserial* /dev/cu.SLAB* \
                /dev/tty.usbmodem* /dev/tty.usbserial* 2>/dev/null || true)"
    if [ -n "$FOUND" ]; then echo "$FOUND" | sed 's/^/  /'
    else echo "  (no USB serial devices attached)"; fi
    [ -t 0 ] || die "port $PORT does not exist"
    PORT=""
    RETRY=$((${RETRY:-0} + 1)); [ "$RETRY" -lt 20 ] || die "giving up looking for a port"
done
case "$PORT" in
  /dev/tty.*) [ -e "/dev/cu.${PORT#/dev/tty.}" ] \
      && echo "hint: if esptool hangs, try /dev/cu.${PORT#/dev/tty.} instead" ;;
esac

idf.py -p "$PORT" flash
echo
echo "flashed drone $DRONE_ID ($HOST_IP) on $PORT"
