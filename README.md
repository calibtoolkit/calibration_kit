# calibration_kit

[![Ubuntu 20.04](https://github.com/calibtoolkit/calibration_kit/actions/workflows/ubuntu-focal.yml/badge.svg)](https://github.com/calibtoolkit/calibration_kit/actions/workflows/ubuntu-focal.yml)
[![Ubuntu 18.04](https://github.com/calibtoolkit/calibration_kit/actions/workflows/ubuntu-bionic.yml/badge.svg)](https://github.com/calibtoolkit/calibration_kit/actions/workflows/ubuntu-bionic.yml)

`calibration_kit` 是一个常用传感器标定算法集合工具，包含了单双目相机标定、相机-雷达标定、雷达-雷达标定、雷达-IMU标定

## 构建

### 工具链要求

CMake >= 3.12

g++ >= 8

clang >= 7

### 步骤

1、克隆此项目

```shell
$ git clone https://github.com/calibtoolkit/calibration_kit.git
$ cd calibration_kit
```

2、安装编译所需的依赖项

```shell
$ sudo apt install libboost-dev libopencv-dev libeigen3-dev libpcl-dev libceres-dev libyaml-cpp-dev
```

`calibration_kit` 不依赖 `ROS`，但是如果你已经安装了 `ROS`，那么你只需要安装 `ceres` 和 `yaml-cpp`

3、执行构建

```shell
$ cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug 
$ cmake --build build --parallel 4
```

4、执行程序

```shell
$ ./build/calibration_kit
```
