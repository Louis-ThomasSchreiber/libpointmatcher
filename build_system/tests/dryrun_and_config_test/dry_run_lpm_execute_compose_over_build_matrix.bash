#!/bin/bash

clear

# ....path resolution logic........................................................................
_PATH_TO_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
LPM_ROOT_DIR="$(dirname "${_PATH_TO_SCRIPT}")"
cd "${LPM_ROOT_DIR}/../../"

# ====begin========================================================================================
bash lpm_execute_compose_over_build_matrix.bash --fail-fast -- build --dry-run
