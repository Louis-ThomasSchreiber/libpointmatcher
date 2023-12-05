#!/bin/bash -i
#
# Libpointmatcher dependencies installer
#
# Usage:
#   $ source lpm_install_dependencies_libnabo_ubuntu.bash
#
#   $ export OVERRIDE_LIBNABO_CMAKE_INSTALL_PREFIX=( "-D CMAKE_INSTALL_PREFIX=/opt" ) && source lpm_install_dependencies_libnabo_ubuntu.bash
#
# Global:
#   - Read "OVERRIDE_LIBNABO_CMAKE_INSTALL_PREFIX"
#
set -e # Note: we want the installer to always fail-fast (it wont affect the build system policy)

# ....Project root logic...........................................................................
TMP_CWD=$(pwd)

if [[ "$(basename $(pwd))" != "build_system" ]]; then
  cd ../
fi

# ....Load environment variables from file.........................................................
set -o allexport
source ./.env
source ./.env.prompt
set +o allexport

# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# ....Helper function..............................................................................
# import shell functions from utilities library
source ./function_library/prompt_utilities.bash
source ./function_library/terminal_splash.bash
source ./function_library/general_utilities.bash

# Set environment variable NBS_IMAGE_ARCHITECTURE
source ./lpm_utility_script/lpm_export_which_architecture.bash

# ....Override.....................................................................................
declare -a DEFAULT_LIBNABO_CMAKE_INSTALL_PREFIX=( "-D CMAKE_INSTALL_PREFIX=${NBS_LIB_INSTALL_PATH:?err}" )
declare -a OVERRIDE_LIBNABO_CMAKE_INSTALL_PREFIX
declare -a LIBNABO_CMAKE_INSTALL_PREFIX=( "${OVERRIDE_LIBNABO_CMAKE_INSTALL_PREFIX[@]:-${DEFAULT_LIBNABO_CMAKE_INSTALL_PREFIX[@]}}" )


# ====Begin========================================================================================
SHOW_SPLASH_IDU="${SHOW_SPLASH_IDU:-true}"

if [[ "${SHOW_SPLASH_IDU}" == 'true' ]]; then
  norlab_splash "${NBS_SPLASH_NAME}" "https://github.com/${NBS_REPOSITORY_DOMAIN}/${NBS_REPOSITORY_NAME}"
fi

print_formated_script_header "lpm_install_dependencies_libnabo_ubuntu.bash (${NBS_IMAGE_ARCHITECTURE})" "${NBS_LINE_CHAR_INSTALLER}"

# .................................................................................................
teamcity_service_msg_blockOpened "Install Libpointmatcher dependencies › Libnabo"
# https://github.com/ethz-asl/libnabo

print_msg "Create required dir structure"
mkdir -p "${NBS_LIB_INSTALL_PATH}"

## Note:
#   - ANN is not mentioned in doc because it's only required for `make test` benchmarks
#   - Leave it commented in code for future references
## ANN is a library written in C++, which supports data structures and algorithms for both exact and approximate nearest neighbor searching in arbitrarily high dimensions.
## https://www.cs.umd.edu/~mount/ANN/
#cd "${NBS_LIB_INSTALL_PATH}"
#wget https://www.cs.umd.edu/~mount/ANN/Files/1.1.2/ann_1.1.2.tar.gz
#tar xzf ann_1.1.2.tar.gz
#cd ann_1.1.2/
#make linux-g++
#sudo cp lib/libANN.a /usr/local/lib/
#sudo cp include/ANN/ANN.h /usr/local/include/
## shellcheck disable=SC2103
#cd ..
#
#
## Note:
#   - FLANN is not mentioned in doc because it's only required for `make test` benchmarks
#   - Leave it commented in code for future references
## Fast Library for Approximate Nearest Neighbors - development
## FLANN is a library for performing fast approximate nearest neighbor searches in high dimensional spaces.
## https://github.com/flann-lib/flann
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        libflann-dev \
#    && sudo rm -rf /var/lib/apt/lists/*

cd "${NBS_LIB_INSTALL_PATH}"
git clone https://github.com/ethz-asl/libnabo.git &&
  cd libnabo &&
  mkdir build && cd build

# git checkout 1.0.7


teamcity_service_msg_compilationStarted "cmake"

## (Priority) inprogress: investigate?? (ref task NMO-402 fix: unstable compilation issue)
## ToDo: Add mention about 'CMAKE_INSTALL_PREFIX' in the doc install step as a fix
#cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo \
# -D CMAKE_INSTALL_PREFIX=${NBS_LIB_INSTALL_PATH} \
# "${NBS_LIB_INSTALL_PATH}/libnabo" &&
#  make -j $(nproc) &&
#  make test &&
#  sudo make install


cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo ${LIBNABO_CMAKE_INSTALL_PREFIX[@]} \
  "${NBS_LIB_INSTALL_PATH}/libnabo" &&
  make -j $(nproc) &&
  sudo make install

# (NICE TO HAVE) ToDo: refactor (ref task NMO-428 refactor: drop libnabo `make test` step after libnabo-build-system deployment)
#  make test &&

teamcity_service_msg_compilationFinished

teamcity_service_msg_blockClosed

echo " " && print_msg_done "Libpointmatcher dependencies installed"
print_formated_script_footer "lpm_install_dependencies_libnabo_ubuntu.bash (${NBS_IMAGE_ARCHITECTURE})" "${NBS_LINE_CHAR_INSTALLER}"

# ====Teardown=====================================================================================
cd "${TMP_CWD}"
