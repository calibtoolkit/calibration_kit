#ifndef LIDAR_ALIGN_SENSORS_H_
#define LIDAR_ALIGN_SENSORS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <random>

#include "lidar_align/conf_parser.h"
#include "lidar_align/transform.h"

struct PointXYZIT {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                          float, intensity,
                                          intensity)(double, timestamp, timestamp))

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Pointcloud;
typedef pcl::PointCloud<PointXYZIT> LoaderPointcloud;
typedef LoaderPointcloud::Ptr LoaderPointcloudPtr;

typedef pcl::PointNormal PointXYZN;
typedef pcl::PointCloud<PointXYZN> PointCloudXYZN;
typedef PointCloudXYZN::Ptr PointCloudXYZNPtr;

typedef double Timestamp;  // the timestamp set as sec

namespace lidar_align {

    class OdomTformData {
    public:
        OdomTformData(Timestamp timestamp, Transform T_o0_ot);

        Transform &getTransform();

        Timestamp &getTimestamp();

    private:
        Transform T_o0_ot_;
        Timestamp timestamp_;
    };

    class Odom {
    public:
        void addTransformData(Timestamp &timestamp,
                              Transform &transform);

        Transform getOdomTransform(Timestamp timestamp,
                                   size_t start_idx = 0,
                                   size_t *match_idx = nullptr);

        Transform getOdomTransformOfidx(size_t idx) {
            return data_[idx].getTransform();
        }

        bool empty() { return data_.empty(); }

        int getOdomTransformSize() {
            return data_.size();
        }

    private:
        std::vector<OdomTformData> data_;
    };

    void LidarMotionCompensation(Transform &extr, LoaderPointcloudPtr org_pc, PointCloudXYZNPtr comp_pc,
                                 double min_time, double max_time, Transform &start_pose, Transform &end_pose);

}  // namespace lidar_align

#endif
