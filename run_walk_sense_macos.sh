#!/usr/bin/env bash
# From repo root: run the Flutter app on macOS.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
exec "$ROOT/walk_sense_app/run_macos.sh" "$@"
