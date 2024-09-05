#!/usr/bin/env bash
# set -x
# set -e

# 获取项目根路径
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

# 创建第三方库存放目录 3rdparty
if [ ! -d PROJECT_DIR/"3rdparty" ]; then
    mkdir -p ${PROJECT_DIR}"/3rdparty"
fi
TRDPARTY_DIR="${PROJECT_DIR}/3rdparty"

sudo apt-get -y install python3-catkin-tools

# mavros
sudo apt -y install ros-noetic-mavros

# PCL >= 1.6
echo "Installing PCL library..."
sudo apt -y install libpcl-dev

# eigen>= 3.3.4
echo "Check eigen library..."
if [ ! -d "${TRDPARTY_DIR}/eigen-3.4.0" ]; then
    echo "Downloading eigen..."
    wget -O eigen3.zip https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
    unzip -q eigen3.zip -d "${TRDPARTY_DIR}"
    pushd "${TRDPARTY_DIR}/eigen-3.4.0"
    mkdir build
    cd build
    cmake -DBUILD_SHARED_LIBS=TRUE ..
    make -j8
    sudo make install
    sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
    popd
    rm eigen3.zip
else
    echo "eigen is already installed."
fi

# ceres<= 2.1.0
echo "Check ceres library..."
if [ ! -d "${TRDPARTY_DIR}/ceres-solver-2.1.0" ]; then
    echo "Downloading ceres..."
    sudo apt-get -y install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
    wget -O ceres-solver.zip https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.zip
    unzip -q ceres-solver.zip -d "${TRDPARTY_DIR}"
    pushd "${TRDPARTY_DIR}/ceres-solver-2.1.0"
    mkdir build
    cd build
    cmake -DBUILD_SHARED_LIBS=TRUE ..
    make -j8
    sudo make install
    popd
    rm ceres-solver.zip
else
    echo "ceres is already installed."
fi

# OpenVINS
OPENVINS_DIR="${PROJECT_DIR}/src/tasks/openvins"
if [ ! -d "${OPENVINS_DIR}" ]; then
    echo "Downloading OpenVINS..."
    git clone https://github.com/rpng/open_vins.git "${OPENVINS_DIR}"
    cd ${OPENVINS_DIR}
    pushd "${PROJECT_DIR}"
    catkin_make --source ${PROJECT_DIR}/src/tasks/openvins -j4
    popd
else
    echo "OpenVINS is already installed."
fi

popd >/dev/null

set +x
echo ""
echo "************************************"
echo "X152b setup completed successfully!"
echo "************************************"