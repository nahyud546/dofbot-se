#!/usr/bin/env bash
# Source this file from each terminal before running the T6 ROS nodes.
_t6_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
_t6_ws="${_t6_root}/workspaces/dofbot_ws"
source /opt/ros/humble/setup.bash
# Colcon's installed console scripts use /usr/bin/python3 even when a virtual
# environment is active. Put the active venv on PYTHONPATH so those scripts can
# import the same dependencies as `python -m ...`.
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    _t6_venv_site="${VIRTUAL_ENV}/lib/python3.10/site-packages"
    if [[ ! -d "${_t6_venv_site}" ]]; then
        echo "T6: virtualenv site-packages not found: ${_t6_venv_site}" >&2
        return 1
    fi
    export PYTHONPATH="${_t6_venv_site}:${PYTHONPATH:-}"
else
    echo "T6: activate ../.venv before sourcing setup_t6.bash" >&2
    return 1
fi
# The ONNX CPU runtime is already installed in the venv. Prevent Ultralytics
# from trying to install optional GPU packages while processing the first frame.
export YOLO_AUTOINSTALL=False
export AMENT_PREFIX_PATH="${_t6_ws}/install_t6_regular/dofbot_yolov11:${_t6_ws}/install_t6_regular/dofbot_sorting_3d:${_t6_ws}/install/dofbot_info:${_t6_ws}/install/dofbot_interface:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="${_t6_ws}/install_t6_regular/dofbot_yolov11/lib/python3.10/site-packages:${_t6_ws}/install_t6_regular/dofbot_sorting_3d/lib/python3.10/site-packages:${_t6_ws}/install/dofbot_interface/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${_t6_ws}/install/dofbot_info/lib:${_t6_ws}/install/dofbot_interface/lib:${LD_LIBRARY_PATH:-}"
unset _t6_ws _t6_root _t6_venv_site
