#!/usr/bin/env bash

# Compatibility wrapper. The maintained FaceMode test entry lives under scripts/navi.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

exec "${ROOT_DIR}/scripts/navi/map_aim_point_test.sh" "$@"
