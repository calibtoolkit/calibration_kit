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


typedef pcl::PointNormal PointXYZN;
typedef pcl::PointCloud<PointXYZN> PointCloudXYZN;
typedef PointCloudXYZN::Ptr PointCloudXYZNPtr;

typedef pcl::PointXYZINormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

void icp_match(const PointCloudPtr& src, const PointCloudPtr& base, Eigen::Matrix4d &trans) {
  pcl::NormalEstimationOMP<PointT, PointT> norm_est;
  pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());

  norm_est.setNumberOfThreads(8);
  norm_est.setSearchMethod(kdtree);
  norm_est.setKSearch(20);

  norm_est.setInputCloud(src);
  norm_est.compute(*src);
  norm_est.setInputCloud(base);
  norm_est.compute(*base);

  pcl::IterativeClosestPointWithNormals<PointT, PointT, float> icp;
//  pcl::IterativeClosestPoint<PointT, PointT, float> icp;
  icp.setInputSource(src);
  icp.setInputTarget(base);

  std::cout<<"source points size "<<src->size()<<std::endl;
  std::cout<<"target points size "<<base->size()<<std::endl;

  icp.setMaxCorrespondenceDistance(5.0);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-4);
  icp.setEuclideanFitnessEpsilon(1e-3);

  PointCloudPtr src_registered(new PointCloud);
  Eigen::Matrix4f init_pose = trans.cast<float>();

  icp.align(*src_registered, init_pose);

  std::cout<<"align icp final"<<std::endl;

  if (icp.hasConverged()) {
    trans = icp.getFinalTransformation().cast<double>();
  } else {

    std::cout << "align failed" << std::endl;
  }
}

void lidar2lidarCalibration(const std::string& path1, const std::string& path2)
{
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_pts(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_pts(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::io::loadPCDFile(path1, *source_pts);
  pcl::io::loadPCDFile(path2, *target_pts);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud<pcl::PointXYZINormal>(*source_pts, *source_pts, indices);
  pcl::removeNaNFromPointCloud<pcl::PointXYZINormal>(*target_pts, *target_pts, indices);

  pcl::VoxelGrid<pcl::PointXYZINormal> filter;
  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  filter.setInputCloud(source_pts);
  filter.filter(*source_pts);
  filter.setInputCloud(target_pts);
  filter.filter(*target_pts);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;

  sor.setInputCloud(source_pts);
  sor.setMeanK(10);
  sor.setStddevMulThresh(1.0);
  sor.filter(*source_pts);

  sor.setInputCloud(target_pts);
  sor.filter(*target_pts);

  std::cout<<"load pcd"<<std::endl;

  Eigen::Matrix4d tran_mat_icp;
  Eigen::Matrix4d tran_mat_teaser;

  tran_mat_icp.setIdentity();

  icp_match(source_pts, target_pts, tran_mat_icp);

  std::cout<<"the calibration result is "<<std::endl<<tran_mat_icp<<std::endl;

  PointCloudPtr transed_cloud(new PointCloud);
  pcl::transformPointCloud(*source_pts, *transed_cloud, tran_mat_icp);

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("trans_viewer"));
  viewer->setBackgroundColor(0, 0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> source_color_handle(target_pts, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> target_color_handle(transed_cloud, 0, 255, 0);

  viewer->addPointCloud(target_pts, source_color_handle, "source");
  viewer->addPointCloud(transed_cloud, target_color_handle, "target");

  viewer->spin();
  //https://github.com/PointCloudLibrary/pcl/issues/172
  //viewer windows can't close
  viewer->close();
}
