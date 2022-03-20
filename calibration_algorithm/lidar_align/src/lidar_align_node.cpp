#include "lidar_align/aligner.h"
#include "lidar_align/conf_parser.h"
#include "lidar_align/loader.h"

using namespace lidar_align;

void lidarToIMUCalibration(const std::string& conf_file)
{
    ConfParser conf_parser;
    conf_parser.read_config(conf_file);
    conf_parser.print_config();

    KeyValues conf_data = conf_parser.get_config();
    Loader loader = Loader();
    loader.loadPoseAndPointCloud(conf_data);

    Aligner aligner = Aligner(conf_data);
    aligner.align(loader.odom, loader.pcd_dirs_, loader.pcd_timestamp_);
}
