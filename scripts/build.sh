#!/usr/bin/env bash
# set -x
# set -e

# 获取项目根路径
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../" && pwd )"

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

# v4l2
sudo apt -y install libv4l-dev

# eigen == 3.3.7
echo "Check eigen library..."
if [ ! -d "${TRDPARTY_DIR}/eigen-3.3.7" ]; then
    echo "Downloading eigen=3.3.7 ..."
    wget -O eigen3.3.7.zip https://emnavi-doc-img.oss-cn-beijing.aliyuncs.com/emnavi_video/intro/eigen3.3.7.zip
    # wget -O eigen3.3.7.zip https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip
    unzip -q eigen3.3.7.zip -d "${TRDPARTY_DIR}"
    pushd "${TRDPARTY_DIR}/eigen-3.3.7"
    mkdir build
    cd build
    cmake -DBUILD_SHARED_LIBS=TRUE ..
    make -j8
    sudo make install
    sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
    popd
    rm eigen3.3.7.zip
else
    echo "eigen-3.3.7 is already installed."
fi

# ceres<= 2.1.0
echo "Check ceres library..."
if [ ! -d "${TRDPARTY_DIR}/ceres-solver-2.1.0" ]; then
    echo "Downloading ceres=2.1.0 ..."
    sudo apt-get -y install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
    wget -O ceres-solver-2.1.0.zip https://emnavi-doc-img.oss-cn-beijing.aliyuncs.com/emnavi_video/intro/ceres-solver-2.1.0.zip
    # wget -O ceres-solver-2.1.0.zip https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.zip
    unzip -q ceres-solver-2.1.0.zip -d "${TRDPARTY_DIR}"
    pushd "${TRDPARTY_DIR}/ceres-solver-2.1.0"
    mkdir build
    cd build
    cmake -DBUILD_SHARED_LIBS=TRUE ..
    make -j8
    sudo make install
    popd
    rm ceres-solver-2.1.0.zip
else
    echo "ceres-2.1.0 is already installed."
fi