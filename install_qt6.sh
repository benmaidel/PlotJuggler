#!/usr/bin/env bash
# Installs the exact Qt version PJ4 builds against into ./.qt via aqtinstall.
#
# SINGLE SOURCE OF TRUTH for the Qt version: build.sh, run.sh and CI all expect
# Qt at .qt/${QT_VERSION}/${QT_ARCH_DIR} but only declare it here. To upgrade,
# bump QT_VERSION below (and the matching paths/cache-keys in build.sh, run.sh,
# CMakeLists.txt, and .github/workflows/*). See docs/QT_NOTES.md for the rationale
# behind the current pin and what changed since 6.8. The host-specific arch folder
# (gcc_64 on Linux, macos on macOS) is resolved by platform_env.sh.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/platform_env.sh"

QT_VERSION="6.11.1"
QT_DIR="${SCRIPT_DIR}/.qt/${QT_VERSION}/${QT_ARCH_DIR}"

if [[ -d "$QT_DIR" ]]; then
  echo "Qt ${QT_VERSION} already installed at ${QT_DIR}"
  echo "export CMAKE_PREFIX_PATH=${QT_DIR}"
  exit 0
fi

if ! command -v aqt &>/dev/null; then
  echo "Installing aqtinstall..."
  pip install 'aqtinstall>=3.3'  # >=3.3 knows about Qt 6.11.x
fi

echo "Installing Qt ${QT_VERSION} (${AQT_HOST}/${AQT_ARCH}) via aqtinstall..."
# No add-on Qt modules needed (PJ4 uses only desktop-default modules; charts is
# replaced by the vendored Qwt, websockets is unused).
#
# Retry with backoff: aqt intermittently picks a mirror that is missing the
# metadata checksum ("Failed to download checksum ... Failed to locate XML data
# for Qt version"). It's transient — a retry usually lands on a healthy mirror.
attempt=0
until aqt install-qt "$AQT_HOST" desktop "$QT_VERSION" "$AQT_ARCH" --outputdir "${SCRIPT_DIR}/.qt"; do
  attempt=$((attempt + 1))
  if [[ "$attempt" -ge 5 ]]; then
    echo "aqt failed after ${attempt} attempts" >&2
    exit 1
  fi
  echo "aqt attempt ${attempt} failed (transient mirror?); retrying in $((attempt * 15))s..."
  sleep $((attempt * 15))
done

echo ""
echo "Qt ${QT_VERSION} installed at ${QT_DIR}"
echo ""
echo "To use it, run:"
echo "  export CMAKE_PREFIX_PATH=${QT_DIR}"
