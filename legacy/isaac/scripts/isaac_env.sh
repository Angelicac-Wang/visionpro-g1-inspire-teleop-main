#!/usr/bin/env bash
# Shared Isaac Sim environment fixes for non-owner users of unitree_sim_env.

isaac_prepare_env() {
  local python_bin="${1:?python binary required}"

  local numpy_version
  numpy_version="$("${python_bin}" -c 'import numpy; print(numpy.__version__)' 2>/dev/null || true)"
  if [[ "${numpy_version}" == 2.* ]]; then
    echo "Detected NumPy ${numpy_version}; installing NumPy 1.x for Pinocchio compatibility."
    "${python_bin}" -m pip install --user "numpy<2"
  fi

  export ISAAC_PORTABLE_ROOT="${ISAAC_PORTABLE_ROOT:-${HOME}/.local/share/isaac-sim-portable}"
  mkdir -p "${ISAAC_PORTABLE_ROOT}"
  export ISAAC_KIT_ARGS="${ISAAC_KIT_ARGS:---portable-root ${ISAAC_PORTABLE_ROOT}}"
}
