#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H

#include "lidar_align/loader.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <string>

#include "lidar_align/sensors.h"
#include "lidar_align/transform.h"

#include "proj_api.h"

namespace lidar_align {

    static inline int GetUtmZoneID(const double &lon) { return static_cast<int>(lon / 6) + 31; }

    static inline int WGS84ToUTM(double *lon, double *lat, const int &id) {
        static int zone_id = id;
        static projPJ pj_wgs84 = pj_init_plus("+proj=latlong +ellps=WGS84");
        static projPJ pj_utm = pj_init_plus("+proj=utm +zone=50 +ellps=WGS84");
        if (id != zone_id) {
            zone_id = id;
            std::stringstream stream;
            stream << "+proj=utm +zone=" << zone_id << " +ellps=WGS84" << std::endl;
            pj_utm = pj_init_plus(stream.str().c_str());
        }

        *lon *= M_PI / 180;
        *lat *= M_PI / 180;
        int ret = pj_transform(pj_wgs84, pj_utm, 1, 1, lon, lat, NULL);
        return ret;
    }

    Loader::Loader() {}

    bool Loader::loadPoseAndPointCloud(KeyValues &conf_data) {
        std::ifstream fp(conf_data["pose_dir"].c_str());
        Transform init_pose_inv;
        if (!fp.eof()) {
            std::string line;
            bool first_frame = true;
            while (std::getline(fp, line)) {
                std::vector<std::string> split_str;
                boost::split(split_str, line, boost::is_any_of("  ,\t"));
                // add one judge condition

                if (split_str.size() == 8) {
                    Transform::Rotation quat(
                            std::atof(split_str[7].c_str()), std::atof(split_str[4].c_str()),
                            std::atof(split_str[5].c_str()), std::atof(split_str[6].c_str()));
                    Transform::Translation trans(std::atof(split_str[1].c_str()),
                                                 std::atof(split_str[2].c_str()),
                                                 std::atof(split_str[3].c_str()));

                    int zone_id = GetUtmZoneID(trans[0]);
                    double lon = trans[0];
                    double lat = trans[1];
                    WGS84ToUTM(&lon, &lat, zone_id);
                    trans[0] = lon;
                    trans[1] = lat;
                    Transform pose = Transform(trans, quat);

                    if (first_frame) {
                        init_pose_inv = pose.inverse();
                        first_frame = false;
                    }
                    pose = init_pose_inv * pose;
                    double timestamp = std::atof(split_str[0].c_str());
                    odom.addTransformData(timestamp, pose);
                } else {
                    std::cerr << "The format of poses.txt is wrong." << std::endl;
                }
            }
        }

        std::string dir(conf_data["pcd_path"]);
        if (dir[dir.size() - 1] != '/') {
            dir += "/";
        }

        std::string pcd_path = conf_data["pcd_dir"];
        std::string fileExtension = ".pcd";

        boost::filesystem::directory_iterator itr;
        for (boost::filesystem::directory_iterator itr(pcd_path);
             itr != boost::filesystem::directory_iterator(); ++itr) {
            if (!boost::filesystem::is_regular_file(itr->status())) {
                continue;
            }

            std::string filename = itr->path().filename().string();
            // check if file extension matches
            if (filename.compare(filename.length() - fileExtension.length(),
                                 fileExtension.length(), fileExtension) != 0) {
                continue;
            }

            std::string pcd_timestamp = filename.substr(0, filename.length() - fileExtension.length());
            pcd_timestamp_.push_back(std::atof(pcd_timestamp.c_str()));
            pcd_dirs_.push_back(itr->path().string());
        }

        std::cout<<"odom_size is "<<odom.getOdomTransformSize()<<std::endl;
        std::cout<<"pcd size is "<<pcd_dirs_.size()<<std::endl;

    }

}  // namespace lidar_align
