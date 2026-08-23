#!/usr/bin/env bash
# Copyright 2026 Davide Faconti
# SPDX-License-Identifier: MPL-2.0
#
# Headless visual-verification harness for the 2D view.
#
# Sibling of pj_scene3D/tools/screenshot_3d.sh, same shape: launch plotjuggler4,
# reload a synthetic MCAP through a canned layout containing a 2D dock, grab the
# MediaViewerWidget's framebuffer to a PNG, exit. Its point is that
# MediaViewerWidget is a QRhiWidget, so this is the only end-to-end proof that a
# decoded image actually reaches the screen on a given RHI backend — notably
# METAL on macOS, where the OpenGL path cannot work (Apple GL is frozen at 4.1).
#
# Everything it needs is derived or generated: the fixture (via the shared
# generate_scene3d_fixture.py, which writes /image alongside /tf and /points),
# the layout (from scene2d_screenshot.pj4.xml.in with absolute paths
# substituted), and a staging dir holding the data-source + parser plugins,
# because --plugin-dir accepts exactly ONE directory.
#
# Usage:
#   pj_scene2D/tools/screenshot_2d.sh [-o OUT.png] [-d DELAY_MS] [-k] [-- EXTRA_APP_ARGS...]
#
# Options:
#   -o PATH   screenshot destination (default: <repo>/build/scene2d_screenshot.png)
#   -d MS     ms to wait before the grab (default 9000: the layout load runs on a
#             worker, and the scene layer restore retries until its topic arrives)
#   -k        keep the app's stderr log instead of printing a tail of it
#
# Anything after `--` is forwarded to the app. Playback is deliberately NOT
# started, so the grab always lands on the fixture's first sample; the test card
# is time-invariant anyway, so successive runs are pixel-comparable.
#
# Environment overrides:
#   PJ_PLUGIN_SRC_DIRS  colon-separated dirs to stage plugins from (default: the
#                       per-plugin Release/bin dirs of ../pj-official-plugins)
#   PJ_FIXTURE          fixture path (default <repo>/build/scene2d_fixture.mcap)
#
# WHAT A CORRECT RESULT LOOKS LIKE: the fixture's test card is asymmetric on
# purpose — eight vertical colour bars (red leftmost, magenta rightmost), a thick
# orange diagonal running from the TOP-LEFT to the BOTTOM-RIGHT, and one filled
# orange square in the BOTTOM-LEFT corner, letterboxed on the viewer's clear
# colour. A vertically flipped render turns the diagonal into a "/", a mirrored
# one puts red on the right, and an R/B channel swap turns the leftmost bar blue.
# Anything black is a renderer failure, not a harness artifact.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
source "${REPO_DIR}/platform_env.sh"

QT_VERSION="6.11.1"
BIN="${REPO_DIR}/build/pj_app/plotjuggler4"
# One generator feeds both harnesses; it writes /image by default.
GENERATOR="${REPO_DIR}/pj_scene3D/tools/generate_scene3d_fixture.py"
LAYOUT_TEMPLATE="${SCRIPT_DIR}/scene2d_screenshot.pj4.xml.in"

FIXTURE="${PJ_FIXTURE:-${REPO_DIR}/build/scene2d_fixture.mcap}"
OUT_PNG="${REPO_DIR}/build/scene2d_screenshot.png"
DELAY_MS=9000
KEEP_LOG=0

while getopts ":o:d:kh" opt; do
  case "${opt}" in
    o) OUT_PNG="${OPTARG}" ;;
    d) DELAY_MS="${OPTARG}" ;;
    k) KEEP_LOG=1 ;;
    h) sed -n '6,45p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "screenshot_2d.sh: unknown option -${OPTARG}" >&2; exit 2 ;;
  esac
done
shift $((OPTIND - 1))

[ -x "${BIN}" ] || { echo "screenshot_2d.sh: no app binary at ${BIN} — run ./build.sh first." >&2; exit 1; }

# --- 1. Fixture ------------------------------------------------------------
if [ ! -f "${FIXTURE}" ]; then
  echo "screenshot_2d.sh: generating fixture ${FIXTURE}"
  python3 "${GENERATOR}" "${FIXTURE}" --verify \
    || { echo "screenshot_2d.sh: fixture generation failed (pip3 install --break-system-packages mcap?)" >&2; exit 1; }
fi

# --- 2. Plugin staging -----------------------------------------------------
# --plugin-dir takes ONE directory, and the app's discovery walks it recursively
# reading each DSO's embedded manifest — so a staging dir of symlinks to the
# individual plugin binaries is enough, and it keeps unrelated build artifacts
# (test binaries, object files) out of the scan.
PLUGINS_ROOT="${REPO_DIR}/../pj-official-plugins/build"
DEFAULT_SRC_DIRS="${PLUGINS_ROOT}/data_load_mcap/Release/bin:${PLUGINS_ROOT}/parser_ros/Release/bin:${PLUGINS_ROOT}/all/Release/bin"
STAGE_DIR="${REPO_DIR}/build/scene2d_harness_plugins"
rm -rf "${STAGE_DIR}"
mkdir -p "${STAGE_DIR}"

staged=0
IFS=':' read -r -a src_dirs <<< "${PJ_PLUGIN_SRC_DIRS:-${DEFAULT_SRC_DIRS}}"
for dir in "${src_dirs[@]}"; do
  [ -d "${dir}" ] || continue
  for lib in "${dir}"/*.dylib "${dir}"/*.so; do
    [ -e "${lib}" ] || continue
    ln -sf "${lib}" "${STAGE_DIR}/$(basename "${lib}")"
    staged=$((staged + 1))
  done
done
if [ "${staged}" -eq 0 ]; then
  echo "screenshot_2d.sh: no plugin binaries found in ${PJ_PLUGIN_SRC_DIRS:-${DEFAULT_SRC_DIRS}}" >&2
  echo "screenshot_2d.sh: build them with: (cd ${PLUGINS_ROOT}/.. && ./build.sh)" >&2
  exit 1
fi
echo "screenshot_2d.sh: staged ${staged} plugin binary/ies in ${STAGE_DIR}"

# --- 3. Layout -------------------------------------------------------------
# The layout embeds ABSOLUTE paths (both in <fileInfo filename> and inside the
# plugin's own config JSON), so it is regenerated on every run rather than
# committed.
LAYOUT="${REPO_DIR}/build/scene2d_screenshot.pj4.xml"
FIXTURE_ABS="$(cd "$(dirname "${FIXTURE}")" && pwd)/$(basename "${FIXTURE}")"
sed -e "s|@MCAP_PATH@|${FIXTURE_ABS}|g" \
    -e "s|@DATASET_SOURCE@|$(basename "${FIXTURE_ABS}")|g" \
    "${LAYOUT_TEMPLATE}" > "${LAYOUT}"

# --- 4. Run ----------------------------------------------------------------
# Qt plugin discovery must point at the bundled Qt only: a stale QT_PLUGIN_PATH
# from the user's shell makes Qt load a foreign platform/TLS plugin and crash
# (same reasoning as run.sh).
export QT_PLUGIN_PATH="${REPO_DIR}/.qt/${QT_VERSION}/${QT_ARCH_DIR}/plugins"
if [ "$(uname -s)" != "Darwin" ]; then
  export QT_IM_MODULE=""
fi
# qt.rhi.general reports the chosen backend and adapter, which is exactly what a
# backend-port check wants in the log; pj.* carries the app's own diagnostics. The
# other qt.rhi.* categories (notably qt.rhi.rub) bury those few lines under
# per-frame buffer traffic, so they stay off unless you ask for them.
export QT_LOGGING_RULES="${QT_LOGGING_RULES:-pj.*=true;qt.rhi.general=true}"

rm -f "${OUT_PNG}"
LOG="${REPO_DIR}/build/scene2d_screenshot.log"

set +e
"${BIN}" --nosplash \
         --plugin-dir "${STAGE_DIR}" \
         --layout "${LAYOUT}" \
         --screenshot "${OUT_PNG}" \
         --screenshot-delay "${DELAY_MS}" \
         "$@" >"${LOG}" 2>&1
status=$?
set -e

# --- 5. Report -------------------------------------------------------------
echo
echo "screenshot_2d.sh: app exited with ${status}; log: ${LOG}"
if [ "${KEEP_LOG}" -eq 0 ]; then
  echo "--- log highlights ---"
  # These are the lines that tell you WHETHER THE HARNESS ACTUALLY WORKED: the
  # RHI backend actually initialized, a parser-bind for /image (the MCAP was read
  # and classified as kImage), no layer-restore warnings (the layout's scene layer
  # found its topic), and the screenshot verdict.
  grep -Ei "screenshot|parser-bind|object_kind|no DataSource plugin|loadConfig|preset rejected|could not be restored|Metal|Vulkan|OpenGL|pipeline|shader" \
    "${LOG}" || echo "(nothing matched the highlight filter)"
  echo "----------------------"
fi

if [ -f "${OUT_PNG}" ]; then
  echo "screenshot_2d.sh: PNG written: ${OUT_PNG}"
  exit 0
fi
echo "screenshot_2d.sh: NO PNG produced — see ${LOG}" >&2
exit 1
