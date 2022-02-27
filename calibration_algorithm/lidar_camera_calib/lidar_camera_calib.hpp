#include <fstream>
#include <iostream>
#include <string>
#include <cassert>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "gflags/gflags.h"
#include "yaml-cpp/yaml.h"
#include <glog/logging.h>
#include "omp.h"

namespace kit {
namespace tools {

struct PointXYZIT {
    float x;
    float y;
    float z;
    unsigned char intensity;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment


class LidarCameraCalib
{
public:
    double g_yaw = 0;
    double g_pitch = 0;
    double g_roll = 0;
    double g_x = 0;
    double g_y = 0;
    double g_z = 0;

    cv::Mat K, D;
    Eigen::Affine3d cam_extrinsic;
    cv::Mat undistort_image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

public:

    LidarCameraCalib(const std::string &cam_intrinsic_file,
                     const std::string &cam_lidar_extrinsic_file);
    ~LidarCameraCalib();

    void StartCalib(const cv::Mat &image, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_read);

    void StartCalib(const cv::Mat &image, pcl::PointCloud<kit::tools::PointXYZIT>::Ptr &cloud_read);

    bool LoadExtrinsic(const std::string &file_path, Eigen::Affine3d *extrinsic);
    bool LoadIntrinsic(const std::string &intrinsics_path,
                                         cv::Mat *dist_coeffs,
                                         cv::Mat *intrisic_mat);

    void Project();

    static void UpdateZ(int value, void* this_ptr);
    static void UpdateY(int value, void* this_ptr);
    static void UpdateX(int value, void* this_ptr);
    static void UpdateRoll(int value, void* this_ptr);
    static void UpdatePitch(int value, void* this_ptr);
    static void UpdateYaw(int value, void* this_ptr);
};




} // namespace tools
} // namespace kit

POINT_CLOUD_REGISTER_POINT_STRUCT(kit::tools::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      std::uint8_t,
                                      intensity,
                                      intensity)(double, timestamp, timestamp))
