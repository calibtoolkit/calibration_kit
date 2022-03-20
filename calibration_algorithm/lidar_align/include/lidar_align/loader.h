#ifndef LIDAR_ALIGN_LOADER_H_
#define LIDAR_ALIGN_LOADER_H_

#include <pcl/point_types.h>
// #include <ros/ros.h>

#include "lidar_align/conf_parser.h"
#include "lidar_align/sensors.h"

namespace lidar_align {

    class Loader {
    public:
        Loader();

        bool loadPoseAndPointCloud(KeyValues &conf_data);

        std::vector<std::string> pcd_dirs_;
        std::vector<double> pcd_timestamp_;
        Odom odom;
    };
}  // namespace lidar_align

#endif  // LIDAR_ALIGN_ALIGNER_H_
