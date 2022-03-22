# calibration_kit

[![Ubuntu 20.04](https://github.com/calibtoolkit/calibration_kit/actions/workflows/ubuntu-focal.yml/badge.svg)](https://github.com/calibtoolkit/calibration_kit/actions/workflows/ubuntu-focal.yml)
[![Ubuntu 18.04](https://github.com/calibtoolkit/calibration_kit/actions/workflows/ubuntu-bionic.yml/badge.svg)](https://github.com/calibtoolkit/calibration_kit/actions/workflows/ubuntu-bionic.yml)

`calibration_kit` 是一个常用传感器标定算法集合工具，包含了单双目相机标定、相机-激光雷达标定、激光雷达-激光雷达标定、激光雷达-IMU标定。

## 构建

### 工具链要求

CMake >= 3.13

g++ >= 9

clang >= 9

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

## 测试数据

所有测试数据位于test_data目录下，lidar2imu由于数据量过大保存在网盘中

链接：https://pan.baidu.com/s/1NUOQ8msgWU6wxaIIxxaGvQ  提取码：slsd

使用此数据时请确保 `calibration_kit` 启动目录与 `lidar2imu` 数据中的 `.ini` 文件位于相同目录，如果不同，需要将 `.ini` 文件中的相对路径改为绝对路径

## 目录结构及算法入口

`calibration_kit` 使用FTXUI实现命令行界面，`src` 中的代码是界面实现，`calibration_algorithm` 目录中的代码是所有标定算法实现，`src/main.cpp` 中每一个类对应一个界面，在标定界面中点击start后相应类中的 `calibration` 函数会被调用执行标定流程。
