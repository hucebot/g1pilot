#!/usr/bin/env bash
# Shared first-run setup for the bind-mounted Unitree SDKs at /sdk2/.
# Called by both entrypoint.sh (auto-bringup) and run.sh (interactive).
# Idempotent: skips work if already done. Set REBUILD_UNITREE_SDK2=1 to force.

set -u

if [ -f /sdk2/unitree_sdk2_python/setup.py ] && [ ! -d /sdk2/unitree_sdk2_python/unitree_sdk2py.egg-info ]; then
    echo "[sdk_setup] Installing /sdk2/unitree_sdk2_python in editable mode..."
    /home/.base/bin/pip install -e /sdk2/unitree_sdk2_python || \
        echo "[sdk_setup] WARNING: editable install of unitree_sdk2_python failed"
fi

if [ -f /sdk2/unitree_sdk2/CMakeLists.txt ]; then
    if [ "${REBUILD_UNITREE_SDK2:-0}" = "1" ] || [ ! -d /sdk2/unitree_sdk2/build/bin ]; then
        echo "[sdk_setup] Building /sdk2/unitree_sdk2 (C++)..."
        mkdir -p /sdk2/unitree_sdk2/build
        ( cd /sdk2/unitree_sdk2/build && cmake .. && make -j"$(nproc)" && make install && ldconfig ) || \
            echo "[sdk_setup] WARNING: build of unitree_sdk2 failed"
    fi
fi
