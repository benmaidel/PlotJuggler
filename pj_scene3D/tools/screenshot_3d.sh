#!/usr/bin/env bash
# Copyright 2026 Davide Faconti
# SPDX-License-Identifier: MPL-2.0
#
# Headless visual-verification harness for the 3D view.
#
# One command that launches plotjuggler4, reloads a synthetic MCAP fixture
# (TF tree + point cloud) through a canned layout containing a 3D dock, grabs
# the first SceneViewWidget's framebuffer to a PNG, and exits. Intended as the
# repeatable before/after check for each stage of the OpenGL -> QRhi/Metal
# renderer port.
#
# Everything it needs is derived or generated: the fixture (via
# generate_scene3d_fixture.py), the layout (from scene3d_screenshot.pj4.xml.in
# with absolute paths substituted), and a staging dir holding the data-source +
# parser plugins, because --plugin-dir accepts exactly ONE directory.
#
# Usage:
#   pj_scene3D/tools/screenshot_3d.sh [-o OUT.png] [-d DELAY_MS] [-- EXTRA_APP_ARGS...]
#
# Options:
#   -o PATH   screenshot destination (default: <repo>/build/scene3d_screenshot.png)
#   -d MS     ms to wait before the grab (default 9000: the layout load runs on a
#             worker, and the 3D layer restore retries until its topics arrive)
#   -k        keep the app's stderr log instead of printing a tail of it
#
# Anything after `--` is forwarded to the app. Playback is deliberately NOT
# started, so the grab always lands on the fixture's first TF/cloud sample and
# successive runs are pixel-comparable; pass `-- --autoplay` when you want the
# animated pose instead, and accept that the frame is then time-dependent.
#
# Environment overrides:
#   PJ_PLUGIN_SRC_DIRS  colon-separated dirs to stage plugins from (default: the
#                       per-plugin Release/bin dirs of ../pj-official-plugins)
#   PJ_FIXTURE          fixture path (default <repo>/build/scene3d_fixture.mcap)
#
# NOTE ON macOS: the 3D view is intentionally disabled there (SceneViewWidget
# finds no OpenGL 4.5 core context and paints a "3D view unavailable"
# placeholder), so a successful macOS run screenshots THAT placeholder. That is
# the expected result and doubles as a check that the guard still works.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
source "${REPO_DIR}/platform_env.sh"

QT_VERSION="6.11.1"
BIN="${REPO_DIR}/build/pj_app/plotjuggler4"
GENERATOR="${SCRIPT_DIR}/generate_scene3d_fixture.py"
LAYOUT_TEMPLATE="${SCRIPT_DIR}/scene3d_screenshot.pj4.xml.in"

FIXTURE="${PJ_FIXTURE:-${REPO_DIR}/build/scene3d_fixture.mcap}"
OUT_PNG="${REPO_DIR}/build/scene3d_screenshot.png"
DELAY_MS=9000
KEEP_LOG=0

while getopts ":o:d:kh" opt; do
  case "${opt}" in
    o) OUT_PNG="${OPTARG}" ;;
    d) DELAY_MS="${OPTARG}" ;;
    k) KEEP_LOG=1 ;;
    h) sed -n '6,36p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "screenshot_3d.sh: unknown option -${OPTARG}" >&2; exit 2 ;;
  esac
done
shift $((OPTIND - 1))

[ -x "${BIN}" ] || { echo "screenshot_3d.sh: no app binary at ${BIN} — run ./build.sh first." >&2; exit 1; }

# --- 1. Fixture ------------------------------------------------------------
if [ ! -f "${FIXTURE}" ]; then
  echo "screenshot_3d.sh: generating fixture ${FIXTURE}"
  python3 "${GENERATOR}" "${FIXTURE}" \
    || { echo "screenshot_3d.sh: fixture generation failed (pip3 install --break-system-packages mcap?)" >&2; exit 1; }
fi

# --- 2. Plugin staging -----------------------------------------------------
# --plugin-dir takes ONE directory, and the app's discovery walks it
# recursively reading each DSO's embedded manifest — so a staging dir of
# symlinks to the individual plugin binaries is enough, and it keeps unrelated
# build artifacts (test binaries, object files) out of the scan.
PLUGINS_ROOT="${REPO_DIR}/../pj-official-plugins/build"
DEFAULT_SRC_DIRS="${PLUGINS_ROOT}/data_load_mcap/Release/bin:${PLUGINS_ROOT}/parser_ros/Release/bin:${PLUGINS_ROOT}/all/Release/bin"
STAGE_DIR="${REPO_DIR}/build/scene3d_harness_plugins"
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
  echo "screenshot_3d.sh: no plugin binaries found in ${PJ_PLUGIN_SRC_DIRS:-${DEFAULT_SRC_DIRS}}" >&2
  echo "screenshot_3d.sh: build them with: (cd ${PLUGINS_ROOT}/.. && ./build.sh)" >&2
  exit 1
fi
echo "screenshot_3d.sh: staged ${staged} plugin binary/ies in ${STAGE_DIR}"

# --- 3. Layout -------------------------------------------------------------
# The layout embeds ABSOLUTE paths (both in <fileInfo filename> and inside the
# plugin's own config JSON), so it is regenerated on every run rather than
# committed.
LAYOUT="${REPO_DIR}/build/scene3d_screenshot.pj4.xml"
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
# Keep the harness independent of whatever the developer last did in the GUI:
# the update check and the meme splash are both suppressed below, but the
# layout/scene QSettings are shared with the real app by design.
export QT_LOGGING_RULES="${QT_LOGGING_RULES:-pj.*=true}"

rm -f "${OUT_PNG}"
LOG="${REPO_DIR}/build/scene3d_screenshot.log"

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
echo "screenshot_3d.sh: app exited with ${status}; log: ${LOG}"
if [ "${KEEP_LOG}" -eq 0 ]; then
  echo "--- log highlights ---"
  # These are the lines that tell you WHETHER THE HARNESS ACTUALLY WORKED:
  # a parser-bind per topic (the MCAP was read and classified), no restore
  # warnings (the layout's scene layers found their topics), and the screenshot
  # verdict. "unavailable/OpenGL" is the macOS placeholder confirmation.
  grep -Ei "screenshot|parser-bind|no DataSource plugin|loadConfig|preset rejected|could not be restored|unavailable|OpenGL" \
    "${LOG}" || echo "(nothing matched the highlight filter)"
  echo "----------------------"
fi

if [ -f "${OUT_PNG}" ]; then
  echo "screenshot_3d.sh: PNG written: ${OUT_PNG}"
  exit 0
fi
echo "screenshot_3d.sh: NO PNG produced — see ${LOG}" >&2
exit 1
