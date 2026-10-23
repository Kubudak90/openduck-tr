#!/usr/bin/env bash
# OpenDuck tam derleme: build/ altina FCStd, STEP, STL, URDF/MJCF, BOM, gorseller.
#
#   tools/build.sh            # her sey (ekran yoksa xvfb-run ile GUI acar)
#   tools/build.sh --check    # ayrica carpisma kontrolu (~3 dk)
#   FREECAD=/yol/freecad tools/build.sh
#
# FreeCAD 1.0+ gerekir. GUI (renk + gorsel) icin "freecad", yoksa "freecadcmd" kullanilir.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export OPENDUCK_ROOT="$ROOT"
export OPENDUCK_OUT="${OPENDUCK_OUT:-$ROOT/build}"
mkdir -p "$OPENDUCK_OUT"

find_bin() {
  for c in "$@"; do
    if [ -n "$c" ] && command -v "$c" >/dev/null 2>&1; then command -v "$c"; return 0; fi
  done
  return 1
}
FC_GUI="$(find_bin "${FREECAD:-}" freecad FreeCAD /opt/freecad-dl/squashfs-root/usr/bin/freecad || true)"
FC_CMD="$(find_bin "${FREECADCMD:-}" freecadcmd FreeCADCmd /opt/freecad-dl/squashfs-root/usr/bin/freecadcmd || true)"

cd "$ROOT"
if [ -n "$FC_GUI" ]; then
  if [ -z "${DISPLAY:-}" ] && command -v xvfb-run >/dev/null 2>&1; then
    xvfb-run -a -s "-screen 0 1920x1080x24" "$FC_GUI" "$ROOT/tools/build_gui.py" >/dev/null 2>&1 || true
  else
    "$FC_GUI" "$ROOT/tools/build_gui.py" >/dev/null 2>&1 || true
  fi
elif [ -n "$FC_CMD" ]; then
  "$FC_CMD" -c "import runpy; runpy.run_path('$ROOT/tools/build_gui.py')" >/dev/null 2>&1 || true
else
  echo "FreeCAD bulunamadi (FREECAD=/yol/freecad ile belirtin)" >&2; exit 1
fi
cat "$OPENDUCK_OUT/build.log"
grep -q "bitti" "$OPENDUCK_OUT/build.log"

if [ "${1:-}" = "--check" ]; then
  "${FC_CMD:?freecadcmd gerekli}" -c "import runpy; runpy.run_path('$ROOT/cad/check.py', run_name='__main__')" \
    2>/dev/null | sed -n '/^# OpenDuck/,$p'
fi
