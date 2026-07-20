#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec python3 "${ROOT_DIR}/scripts/understand_graph_dashboard.py" \
  --host "${LIBRARY_HOST:-127.0.0.1}" \
  --port "${LIBRARY_PORT:-1037}" \
  "$@"
