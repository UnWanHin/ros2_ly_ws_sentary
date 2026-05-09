#!/usr/bin/env bash

# Compatibility wrapper for the old FaceMode entrypoint.
# Old behavior is official-map X/Y through tf_config.yaml matrix into map frame.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

exec "${ROOT_DIR}/scripts/navi/facemode_cross_matrix.sh" "$@"
