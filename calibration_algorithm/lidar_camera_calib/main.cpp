#include "lidar_camera_calib.hpp"

DEFINE_string(pcd_file, "../test/1.pcd", "pcd file");
DEFINE_string(img_file, "../test/1.jpeg", "image file");
DEFINE_string(cam_lidar_extrinsic_file,
              "../test/front_6mm_extrinsics.yaml",
              "provide camera extrinsic file path");
DEFINE_string(cam_intrinsic_file,
              "../test/front_6mm_intrinsics.yaml",
              "provide camera intrinsic file path");
DEFINE_bool(
    pcd_with_timestamps,
    true,
    "pcd type is 'PointXYZIT' (use true) or 'PointXYZI' (use fasle)? ");

int main(int argc, char **argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::string pcd_file = FLAGS_pcd_file;
    std::string img_file = FLAGS_img_file;
    std::string cam_lidar_extrinsic_file = FLAGS_cam_lidar_extrinsic_file;
    std::string cam_intrinsic_file = FLAGS_cam_intrinsic_file;

    cv::Mat image = cv::imread(img_file);

    kit::tools::LidarCameraCalib tools(cam_intrinsic_file, cam_lidar_extrinsic_file);

    if (FLAGS_pcd_with_timestamps)
    {
        pcl::PointCloud<kit::tools::PointXYZIT>::Ptr cloud_read(
            new pcl::PointCloud<kit::tools::PointXYZIT>());

        if (pcl::io::loadPCDFile(pcd_file, *cloud_read) == -1)
        {
            std::cout << "couldn't read file" << pcd_file << std::endl;
            return -1;
        }
        tools.StartCalib(image, cloud_read);
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>());

        if (pcl::io::loadPCDFile(pcd_file, *cloud) == -1)
        {
            std::cout << "couldn't read file" << pcd_file << std::endl;
            return -1;
        }
        tools.StartCalib(image, cloud);
    }

    return 0;
}
