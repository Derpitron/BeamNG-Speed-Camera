#!/usr/bin/env bash
set -euo pipefail
BEAMNG="${1:-$HOME/.local/share/Steam/steamapps/common/BeamNG.drive/lua}"
WORKSPACE="${2:-$(pwd)}"
OUT_DIR="${WORKSPACE}/.vscode/beamng_stubs"

echo "BeamNG lua: $BEAMNG"
echo "Workspace:  $WORKSPACE"
echo "Stubs out:  $OUT_DIR"

if [ ! -f "${WORKSPACE}/tools/generate_beamng_stubs_ast.py" ]; then
  echo "ERROR: tools/generate_beamng_stubs_ast.py not found in ${WORKSPACE}/tools" >&2
  exit 2
fi

python3 "${WORKSPACE}/tools/generate_beamng_stubs_ast.py" --beamng "$BEAMNG" --out "$OUT_DIR" --workspace "$WORKSPACE"
echo "Regeneration finished. Reload VS Code (Developer: Reload Window) or restart the Lua language server."
