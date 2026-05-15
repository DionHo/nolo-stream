#!/usr/bin/env bash
# Debug session script for NoloStream (Linux / WSL / Git Bash)
# Usage:  ./debug-session.sh [WS_PORT]   (default port 12345)
#         NOBUILD=1 ./debug-session.sh   — skip cargo build
#
# Starts nolostream_server (WS) + miniviz and prints combined labelled output.
# Ctrl-C kills both processes and cleans up.

set -euo pipefail

WS_PORT="${1:-12345}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SERVER_LOG="/tmp/nolo_server_${WS_PORT}.log"
MINIVIZ_LOG="/tmp/nolo_miniviz_${WS_PORT}.log"

# Detect OS for binary extension
if [[ "$(uname -s)" == MINGW* ]] || [[ "$(uname -s)" == CYGWIN* ]] || [[ "$(uname -s)" == MSYS* ]]; then
    BIN_EXT=".exe"
else
    BIN_EXT=""
fi

SERVER_BIN="$ROOT/target/release/nolostream_server${BIN_EXT}"
MINIVIZ_BIN="$ROOT/target/release/miniviz${BIN_EXT}"

SERVER_PID=0
MINIVIZ_PID=0
TAIL_PIDS=()

cleanup() {
    echo ""
    echo "[stop ] killing processes..."
    [[ $SERVER_PID  -gt 0 ]] && kill "$SERVER_PID"  2>/dev/null || true
    [[ $MINIVIZ_PID -gt 0 ]] && kill "$MINIVIZ_PID" 2>/dev/null || true
    for p in "${TAIL_PIDS[@]:-}"; do
        [[ -n "$p" ]] && kill "$p" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "[done ] logs saved to $SERVER_LOG and $MINIVIZ_LOG"
}
trap cleanup EXIT INT TERM

# ----- build ----------------------------------------------------------------
if [[ "${NOBUILD:-}" != "1" ]]; then
    echo "[build] cargo build --release"
    cargo build --release --manifest-path "$ROOT/Cargo.toml"
fi

for bin in "$SERVER_BIN" "$MINIVIZ_BIN"; do
    if [[ ! -x "$bin" ]]; then
        echo "[error] binary not found: $bin  (run without NOBUILD=1)"
        exit 1
    fi
done

# ----- start processes ------------------------------------------------------
> "$SERVER_LOG"
> "$MINIVIZ_LOG"

echo "[start] nolostream_server --ws-listen-at $WS_PORT --debug"
"$SERVER_BIN" --ws-listen-at "$WS_PORT" --debug >"$SERVER_LOG" 2>&1 &
SERVER_PID=$!

sleep 0.5

echo "[start] miniviz --connect ws://127.0.0.1:$WS_PORT"
"$MINIVIZ_BIN" --connect "ws://127.0.0.1:$WS_PORT" >"$MINIVIZ_LOG" 2>&1 &
MINIVIZ_PID=$!

echo "[logs ] streaming — Ctrl-C to stop"
echo ""

# ----- tail both logs with labels -------------------------------------------
tail -f "$SERVER_LOG"  | sed -u 's/^/[server ] /' &
TAIL_PIDS+=($!)
tail -f "$MINIVIZ_LOG" | sed -u 's/^/[miniviz] /' &
TAIL_PIDS+=($!)

# Wait for server to exit (it shouldn't unless device disconnects)
wait "$SERVER_PID"
