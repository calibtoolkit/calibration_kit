FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV HOME=/root SHELL=/bin/bash

RUN sed -i 's#http://archive.ubuntu.com/#http://mirrors.tuna.tsinghua.edu.cn/#' /etc/apt/sources.list;

RUN apt-get update --fix-missing && \
    apt-get install -y libceres-dev libyaml-cpp-dev libboost-dev libopencv-dev libeigen3-dev libpcl-dev git && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://gitee.com/mirrors/ftxui.git && \
    cd ftxui && mkdir build && cd build && cmake .. && make install && \
    cd ../.. && rm -rf ftxui

WORKDIR /fusion



