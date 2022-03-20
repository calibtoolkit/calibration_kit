#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

#include <ceres/ceres.h>
// #include <ros/ros.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <future>
#include <limits>
#include <omp.h>
#include "lidar_align/sensors.h"

namespace lidar_align {

    struct IcpState {
        bool valid;
        int count;
        double error;
        Transform trans;
    };

    class Aligner {
    public:
        struct Config {
            Transform ext_;

            double min_pair_dist = 5.0;
            double min_pair_yaw = 0.3;

            double max_icp_iteration = 50;
            double max_icp_corres_dist = 5.0;
            double min_icp_euclid_eps = 1e-3;
            double min_icp_transform_eps = 1e-4;

            double max_update_iteration = 50;
            double update_termination_dist_eps = 1e-3;
            double update_termination_angle_eps = 1e-3;
            int fix_extrinsic_z = 0;
            double default_optimization_iter = 400;

        };

        Aligner(KeyValues conf_data);

        void align(Odom odom, std::vector<std::string> pcd_dirs, std::vector<double> pcd_timestamps);

        void icp_match(const PointCloudXYZNPtr &src,
                       const PointCloudXYZNPtr &base, IcpState &icp_state);

    private:

        Config config_;

        bool fix_extrinsic_z_;

        std::vector<LoaderPointcloudPtr> original_point_clouds_;
        std::vector<PointCloudXYZNPtr> compensated_point_clouds_;
    };

}  // namespace lidar_align

#endif  // LIDAR_ALIGN_ALIGNER_H_
