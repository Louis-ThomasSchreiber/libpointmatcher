#!/bin/bash -i
#
# Libpointmatcher documentation dependencies installer
#
# Usage:
#   $ bash lpm_install_doc_dependencies_ubuntu.bash
#
set -e # Note: we want the installer to always fail-fast (it wont affect the build system policy)

## skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# ....Project root logic...........................................................................
TMP_CWD=$(pwd)

LPM_PATH=$(git rev-parse --show-toplevel)
cd "${LPM_PATH}/build_system" || exit

# ....Load environment variables from file.........................................................
set -o allexport
source .env
set +o allexport

# ....Helper function..............................................................................
# import shell functions from utilities library
source "${LPM_PATH}/build_system/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"

# Set environment variable NBS_IMAGE_ARCHITECTURE
source "${LPM_PATH}/build_system/lpm_utility_script/lpm_export_which_architecture.bash"

# ====Begin========================================================================================
SHOW_SPLASH_IDDU="${SHOW_SPLASH_IDDU:-true}"

if [[ "${SHOW_SPLASH_IDDU}" == 'true' ]]; then
  norlab_splash "${NBS_SPLASH_NAME}" "https://github.com/${NBS_REPOSITORY_DOMAIN}/${NBS_REPOSITORY_NAME}"
fi

print_formated_script_header "lpm_install_doc_dependencies_ubuntu.bash (${NBS_IMAGE_ARCHITECTURE})" "${NBS_LINE_CHAR_INSTALLER}"


# .................................................................................................
teamcity_service_msg_blockOpened "Install libpointmatcher documentation related dependencies"

## Package required when GENERATE_API_DOC flag is set to true
## Note: 'texlive-full' is ~6GB. 'doxygen-latex' is a slim version tailor made for doxygen code documentation task
sudo apt-get update &&
  sudo apt-get install --assume-yes \
    doxygen \
    doxygen-latex \
  && sudo rm -rf /var/lib/apt/lists/*

teamcity_service_msg_blockClosed

echo " " && print_msg_done "Libpointmatcher documentation related dependencies installed"
print_formated_script_footer "lpm_install_doc_dependencies_ubuntu.bash (${NBS_IMAGE_ARCHITECTURE})" "${NBS_LINE_CHAR_INSTALLER}"
# ====Teardown=====================================================================================
cd "${TMP_CWD}"

