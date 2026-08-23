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
  # Some distros ship only `pip3`, so don't assume the unversioned name exists.
  PIP=""
  for candidate in pip3 pip; do
    if command -v "$candidate" &>/dev/null; then
      PIP="$candidate"
      break
    fi
  done
  if [[ -z "$PIP" ]]; then
    echo "install_qt6.sh: no pip found. Install it (e.g. sudo apt install python3-pip)" >&2
    echo "install_qt6.sh: or install aqtinstall yourself: pipx install aqtinstall" >&2
    exit 1
  fi

  # PEP 668 (Ubuntu 23.04+, Debian 12+, recent Fedora) marks the system Python
  # "externally managed" and refuses a plain `pip install` outright. Retry with
  # the explicit opt-out instead of failing the whole setup. (On older pip the
  # flag does not exist, but there the first attempt already succeeds.)
  if ! "$PIP" install 'aqtinstall>=3.3'; then  # >=3.3 knows about Qt 6.11.x
    echo "install_qt6.sh: plain pip install failed; retrying with --break-system-packages..."
    "$PIP" install --break-system-packages 'aqtinstall>=3.3'
  fi

  # When pip cannot write to the system prefix it installs into the user base,
  # whose bin/ is added to PATH only at login — so it is invisible to this
  # already-running shell and the aqt call below would still fail.
  if ! command -v aqt &>/dev/null; then
    PATH="$(python3 -m site --user-base)/bin:${PATH}"
    export PATH
  fi
fi

if ! command -v aqt &>/dev/null; then
  echo "install_qt6.sh: aqt is installed but not on PATH." >&2
  echo "install_qt6.sh: add it for this shell and permanently, then re-run:" >&2
  echo "  export PATH=\"\$(python3 -m site --user-base)/bin:\$PATH\"" >&2
  exit 1
fi

echo "Installing Qt ${QT_VERSION} (${AQT_HOST}/${AQT_ARCH}) via aqtinstall..."
# One add-on module: qtshadertools, which provides the `qsb` shader baker that
# CMake's qt6_add_shaders() drives. The QRhi-based scene widgets compile their
# GLSL to .qsb at build time, so without it their shaders cannot be produced.
# Everything else PJ4 needs is a desktop-default module (charts is replaced by
# the vendored Qwt, websockets is unused).
#
# Retry with backoff: aqt intermittently picks a mirror that is missing the
# metadata checksum ("Failed to download checksum ... Failed to locate XML data
# for Qt version"). It's transient — a retry usually lands on a healthy mirror.
attempt=0
until aqt install-qt "$AQT_HOST" desktop "$QT_VERSION" "$AQT_ARCH" -m qtshadertools \
  --outputdir "${SCRIPT_DIR}/.qt"; do
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
