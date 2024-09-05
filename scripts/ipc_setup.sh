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
    make
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
    make
    sudo make install
    popd
    rm ceres-solver.zip
else
    echo "ceres is already installed."
fi

# fmt
echo "Check Fmt library..."
# sudo apt install ros-noetic-rosfmt
if [ ! -d "${TRDPARTY_DIR}/fmt" ]; then
    echo "Downloading fmt..."
    wget -O fmt.zip https://github.com/hku-mars/IPC/releases/download/v0.1/fmt.zip
    unzip -q fmt.zip -d "${TRDPARTY_DIR}"
    pushd "${TRDPARTY_DIR}/fmt"
    mkdir build
    cd build
    cmake -DBUILD_SHARED_LIBS=TRUE ..
    make
    sudo make install
    popd
    sudo cp /usr/local/lib/libfmt.so.8 /usr/lib
    rm fmt.zip
else
    echo "fmt is already installed."
fi

# osqp >= 0.6.3
# 如果遇到cmake版本过低限制不让编译，就去OSQP里面的cmakelists修改一下
echo "Check osqp library..."
if [ ! -d "${TRDPARTY_DIR}/osqp" ]; then
    echo "Downloading osqp..."
    git clone --recursive https://github.com/osqp/osqp.git "${TRDPARTY_DIR}/osqp"
    pushd "${TRDPARTY_DIR}/osqp"
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    popd
else
    echo "osqp is already installed."
fi

# osqp-eigen
echo "Check osqp-eigen library..."
if [ ! -d "${TRDPARTY_DIR}/osqp-eigen" ]; then
    echo "Downloading osqp-eigen..."
    git clone https://github.com/robotology/osqp-eigen.git "${TRDPARTY_DIR}/osqp-eigen"
    pushd "${TRDPARTY_DIR}/osqp-eigen"
    mkdir build
    cd build
    cmake ..
    sudo make
    sudo make install
    popd
else
    echo "osqp-eigen is already installed."
fi

# Debug tool
sudo apt-get install libdw-dev
wget https://raw.githubusercontent.com/bombela/backward-cpp/master/backward.hpp
sudo mv backward.hpp /usr/include

# IPC
IPC_DIR="${PROJECT_DIR}/src/tasks/IPC"
if [ ! -d "${IPC_DIR}" ]; then
    echo "Downloading IPC..."
    git clone https://github.com/hku-mars/IPC.git "${IPC_DIR}"
    cd ${IPC_DIR}
    pushd "${PROJECT_DIR}"
    catkin_make --source ${PROJECT_DIR}/src/tasks/IPC
    popd
else
    echo "IPC is already installed."
fi

popd >/dev/null

set +x
echo ""
echo "************************************"
echo "X152b setup completed successfully!"
echo "************************************"