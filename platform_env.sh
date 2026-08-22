#!/usr/bin/env bash
# Shared host-platform detection for the PJ4 build/run helper scripts. SOURCE it
# (do not execute): install_qt6.sh, build.sh, run.sh, and worktree-new.sh all
# `source platform_env.sh` so the Linux-vs-macOS branch lives in exactly one file.
#
# Exports:
#   QT_ARCH_DIR  on-disk arch folder under .qt/<version>/  (gcc_64 | macos)
#   AQT_HOST     aqt `install-qt` host argument            (linux  | mac)
#   AQT_ARCH     aqt `install-qt` arch argument            (linux_gcc_64 | clang_64)
#   JOBS         parallel build jobs for `cmake --build -j`
#
# GOTCHA (macOS): aqt is asked for arch `clang_64`, but it lays the universal
# build down in a folder literally named `macos` — so the install ARG and the
# on-disk FOLDER differ, unlike Linux where both are gcc_64/linux_gcc_64.

case "$(uname -s)" in
  Darwin)
    QT_ARCH_DIR="macos"
    AQT_HOST="mac"
    AQT_ARCH="clang_64"
    ;;
  Linux)
    QT_ARCH_DIR="gcc_64"
    AQT_HOST="linux"
    AQT_ARCH="linux_gcc_64"
    ;;
  *)
    echo "platform_env.sh: unsupported host OS '$(uname -s)' (expected Linux or Darwin)" >&2
    return 1 2>/dev/null || exit 1
    ;;
esac

# nproc is GNU coreutils (Linux); macOS uses sysctl. getconf is the last resort.
JOBS="$( (command -v nproc >/dev/null 2>&1 && nproc) \
         || sysctl -n hw.ncpu 2>/dev/null \
         || getconf _NPROCESSORS_ONLN 2>/dev/null \
         || echo 4 )"

export QT_ARCH_DIR AQT_HOST AQT_ARCH JOBS
