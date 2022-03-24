#!/bin/bash
workdir="$PWD"
echo workdir : ${workdir}
docker run \
--rm --name calib_kit --privileged --net host -w /sensor_fusion -v ${workdir}:/sensor_fusion -it --env="DISPLAY" \
asd741573661/ros-noetic:calib_kit_v1 /bin/bash
docker attach calib_kit
