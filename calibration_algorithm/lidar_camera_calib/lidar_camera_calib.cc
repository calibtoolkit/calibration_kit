/******************************************************************************
 * Copyright 2022 The Shenlan Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "lidar_camera_calib.hpp"
#include <iostream>
#include <cassert>
#include <yaml-cpp/yaml.h>

namespace kit {
namespace tools {

LidarCameraCalib::LidarCameraCalib(const std::string &cam_intrinsic_file, const std::string &cam_lidar_extrinsic_file)
{
    LoadIntrinsic(cam_intrinsic_file, &D, &K);
    LoadExtrinsic(cam_lidar_extrinsic_file, &cam_extrinsic);
}

void LidarCameraCalib::StartCalib(const cv::Mat &image, pcl::PointCloud<kit::tools::PointXYZIT>::Ptr &cloud_read)
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cv::undistort(image, undistort_image, K, D);
    for (const auto &p : cloud_read->points)
    {
        pcl::PointXYZI pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.intensity = p.intensity;
        cloud->push_back(pt);
    }


    int value_yaw = 18000;
    int value_pitch = 18000;
    int value_roll = 18000;
    int value_x = 50;
    int value_y = 50;
    int value_z = 50;
    cv::namedWindow("mainWin", cv::WINDOW_NORMAL);
    cv::moveWindow("mainWin", 200, 200);
    cv::createTrackbar("yaw", "mainWin", &value_yaw, 36000, UpdateYaw, this);
    cv::createTrackbar("pitch", "mainWin", &value_pitch, 36000, UpdatePitch, this);
    cv::createTrackbar("roll", "mainWin", &value_roll, 36000, UpdateRoll, this);
    cv::createTrackbar("x", "mainWin", &value_x, 100, UpdateX, this);
    cv::createTrackbar("y", "mainWin", &value_y, 100, UpdateY, this);
    cv::createTrackbar("z", "mainWin", &value_z, 100, UpdateZ, this);

    Project();
}

void LidarCameraCalib::StartCalib(const cv::Mat &image, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_read)
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*cloud_read, *cloud);
    cv::undistort(image, undistort_image, K, D);

    int value_yaw = 18000;
    int value_pitch = 18000;
    int value_roll = 18000;
    int value_x = 50;
    int value_y = 50;
    int value_z = 50;
    cv::namedWindow("mainWin", cv::WINDOW_NORMAL);
    cv::moveWindow("mainWin", 200, 200);
    cv::createTrackbar("yaw", "mainWin", &value_yaw, 36000, UpdateYaw, this);
    cv::createTrackbar("pitch", "mainWin", &value_pitch, 36000, UpdatePitch, this);
    cv::createTrackbar("roll", "mainWin", &value_roll, 36000, UpdateRoll, this);
    cv::createTrackbar("x", "mainWin", &value_x, 100, UpdateX, this);
    cv::createTrackbar("y", "mainWin", &value_y, 100, UpdateY, this);
    cv::createTrackbar("z", "mainWin", &value_z, 100, UpdateZ, this);

    Project();
}

void LidarCameraCalib::Project()
{
    Eigen::Affine3d Tdelta = Eigen::Affine3d::Identity();
    Tdelta.translation() = Eigen::Vector3d(g_x, g_y, g_z);
    Tdelta.linear() =
        Eigen::AngleAxisd(g_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(g_pitch, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(g_roll, Eigen::Vector3d::UnitY()).toRotationMatrix();

    Eigen::Affine3d cam_extrinsic_final = cam_extrinsic * Tdelta;
    std::cout << "cam_extrinsic_final translation\n"
              << cam_extrinsic_final.translation() << std::endl;
    std::cout << "cam_extrinsic_final rotation\n"
              << cam_extrinsic_final.linear() << std::endl;
    Eigen::Quaterniond q(cam_extrinsic_final.linear());
    std::cout << "cam_extrinsic_final quaternion qx qy qz qw\n"
              << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cam(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud, *cloud_cam, cam_extrinsic_final.inverse());

    std::vector<cv::Point3d> object_points;
    std::vector<double> intensity_points;
    std::vector<cv::Point2d> image_points;
    for (const auto &point : cloud_cam->points) {
        object_points.emplace_back(point.x, point.y, point.z);
        intensity_points.emplace_back(point.intensity);
    }
    cv::Mat tmp_D = cv::Mat::zeros(D.size(), D.type());
    cv::projectPoints(object_points, cv::Mat::zeros(3, 1, CV_32FC1),
                      cv::Mat::zeros(3, 1, CV_32FC1), K, tmp_D, image_points);
    assert(object_points.size() == image_points.size());
    //assert(object_points[0].x == image_points[0].x);
    std::cout << object_points[0].x - image_points[0].x << std::endl;

    cv::Mat test_image = undistort_image.clone();
    std::cout << "Projected Finished! " << std::endl;

    for (size_t i = 0; i < image_points.size(); ++i) {
        int col = static_cast<int>(std::round(image_points[i].x));
        int row = static_cast<int>(std::round(image_points[i].y));
        const auto &x = cloud_cam->points[i].x;
        const auto &y = cloud_cam->points[i].y;
        const auto &z = cloud_cam->points[i].z;
        auto dist = x * x + y * y + z * z;
        if (z <= 0 || col < 0 || col >= undistort_image.cols || row < 0 ||
            row >= undistort_image.rows || dist > 80 * 80) {
            continue;
        }
        int color = intensity_points[i] * 5;
        if (color > 255) color = 255;

        cv::circle(test_image, cv::Point(col, row), 3, cv::Vec3b(0, color, 0));
    }

    cv::imshow("mainWin", test_image);
    cv::waitKey(0);
    cv::destroyWindow("mainWin");
}

void LidarCameraCalib::UpdateYaw(int value, void *this_ptr)
{
    LidarCameraCalib *calib_ptr = static_cast<LidarCameraCalib *>(this_ptr);
    calib_ptr->g_yaw = (value - 18000) * 0.01 * M_PI / 360;
    calib_ptr->Project();
}

void LidarCameraCalib::UpdatePitch(int value, void* this_ptr)
{
    LidarCameraCalib *calib_ptr = static_cast<LidarCameraCalib *>(this_ptr);
    calib_ptr->g_pitch = (value - 18000) * 0.01 * M_PI / 360;
    calib_ptr->Project();
}

void LidarCameraCalib::UpdateRoll(int value, void* this_ptr)
{
    LidarCameraCalib *calib_ptr = static_cast<LidarCameraCalib *>(this_ptr);
    calib_ptr->g_roll = (value - 18000) * 0.01 * M_PI / 360;
    calib_ptr->Project();
}

void LidarCameraCalib::UpdateX(int value, void* this_ptr)
{
    LidarCameraCalib *calib_ptr = static_cast<LidarCameraCalib *>(this_ptr);
    calib_ptr->g_x = (value - 50) * 0.01;
    calib_ptr->Project();
}

void LidarCameraCalib::UpdateY(int value, void* this_ptr)
{
    LidarCameraCalib *calib_ptr = static_cast<LidarCameraCalib *>(this_ptr);
    calib_ptr->g_y = (value - 50) * 0.01;
    calib_ptr->Project();
}

void LidarCameraCalib::UpdateZ(int value, void* this_ptr)
{
    LidarCameraCalib *calib_ptr = static_cast<LidarCameraCalib *>(this_ptr);
    calib_ptr->g_z = (value - 50) * 0.01;
    calib_ptr->Project();
}

void LidarCameraCalib::LoadExtrinsic(const std::string &file_path, Eigen::Affine3d *extrinsic)
{
    YAML::Node config = YAML::LoadFile(file_path);

    if (config["transform"]) {
        if (config["transform"]["translation"]) {
            extrinsic->translation()(0) =
                config["transform"]["translation"]["x"].as<double>();
            extrinsic->translation()(1) =
                config["transform"]["translation"]["y"].as<double>();
            extrinsic->translation()(2) =
                config["transform"]["translation"]["z"].as<double>();
            if (config["transform"]["rotation"]) {
                double qx = config["transform"]["rotation"]["x"].as<double>();
                double qy = config["transform"]["rotation"]["y"].as<double>();
                double qz = config["transform"]["rotation"]["z"].as<double>();
                double qw = config["transform"]["rotation"]["w"].as<double>();
                extrinsic->linear() =
                    Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
            }
        }
    }
}

void LidarCameraCalib::LoadIntrinsic(const std::string &intrinsics_path, cv::Mat *dist_coeffs, cv::Mat *intrisic_mat)
{
    YAML::Node config = YAML::LoadFile(intrinsics_path);

    if (config["K"] && config["D"])
    {
        std::vector<double> K = config["K"].as<std::vector<double>>();
        std::vector<double> D = config["D"].as<std::vector<double>>();
        *intrisic_mat = cv::Mat(3, 3, cv::DataType<double>::type);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                intrisic_mat->at<double>(i, j) = K[i * 3 + j];
            }
        }
        *dist_coeffs = cv::Mat(5, 1, cv::DataType<double>::type);
        for (int i = 0; i < 5; i++)
        {
            dist_coeffs->at<double>(i) = D[i];
        }
    }
}

}  // namespace tools
}  // namespace kit

