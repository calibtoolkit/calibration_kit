// lidar 2 lidar calibration
#include <chrono>
#include <iostream>
#include <random>
#include <string>
#include <algorithm>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/registration/transformation_estimation_svd.h>




int main(){
  std::string path1 = "/media/brucesz/jzhao/1.pcd";

  pcl::PointCloud<pcl::PointXYZ>::Ptr source_pts(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(path1, *source_pts);

  Eigen::Matrix4d T;
  // clang-format off
  T << 9.96926560e-01,  6.68735757e-02, -4.06664421e-02, -5.15576939e-01,
      -6.61289946e-02, 9.97617877e-01,  1.94008687e-02, -3.87705398e-02,
      4.18675510e-02, -1.66517807e-02,  9.98977765e-01, 1.14874890e-01,
      0,              0,                0,              1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*source_pts, *transed_cloud, T);

  std::string pcd1 = "1_xyz.pcd";
  std::string pcd2 = "2_xyz.pcd";

  pcl::io::savePCDFileBinaryCompressed(pcd1, *source_pts);
  pcl::io::savePCDFileBinaryCompressed(pcd2, *transed_cloud);

  return 0;
}