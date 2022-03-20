#include "lidar_align/sensors.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

namespace lidar_align {

    OdomTformData::OdomTformData(Timestamp timestamp, Transform T_o0_ot)
            : timestamp_(timestamp), T_o0_ot_(T_o0_ot) {}

    Transform &OdomTformData::getTransform() { return T_o0_ot_; }

    Timestamp &OdomTformData::getTimestamp() { return timestamp_; }

    void Odom::addTransformData(Timestamp &timestamp, Transform &T) {
        data_.emplace_back(timestamp, T);
    }

    Transform Odom::getOdomTransform(Timestamp timestamp,
                                     size_t start_idx,
                                     size_t *match_idx) {
        size_t idx = start_idx;

//        std::cout<<"data_ size is "<<data_.size()<<std::endl;

        while ((idx < (data_.size() - 1)) &&
               (timestamp > data_[idx].getTimestamp())) {
            ++idx;
        }
        if (idx > 0) {
            --idx;
        }

        if (match_idx != nullptr) {
            *match_idx = idx;
        }

        // interpolate
        double t_diff_ratio =
                static_cast<double>(timestamp - data_[idx].getTimestamp()) /
                static_cast<double>(data_[idx + 1].getTimestamp() -
                                    data_[idx].getTimestamp());

        Transform::Vector6 diff_vector =
                (data_[idx].getTransform().inverse() * data_[idx + 1].getTransform())
                        .log();
        Transform out =
                data_[idx].getTransform() * Transform::exp(t_diff_ratio * diff_vector);

        return out;
    }

    // the extr is T_imu_lidar
    void LidarMotionCompensation(Transform &extr, LoaderPointcloudPtr org_pc, PointCloudXYZNPtr comp_pc,
                                 double min_time, double max_time, Transform &start_pose, Transform &end_pose) {
        // the pose is imu pose T_w_imu
        Transform start_lidar_pose = start_pose * extr;
        Transform end_lidar_pose = end_pose * extr;

        Transform relative_pose = end_lidar_pose.inverse() * start_lidar_pose;

        Eigen::Vector3f relative_tran = relative_pose.translation_;
        Eigen::Quaternionf relative_quat = relative_pose.rotation_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc(new pcl::PointCloud<pcl::PointXYZ>());

//        temp_pc->resize(org_pc->size());
        double time_range = max_time - min_time;

        for(int i = 0; i < org_pc->size(); i++) {
            auto point = org_pc->points[i];
            double ratio = (max_time - point.timestamp) / time_range;
            Eigen::Quaternionf identity_;
            identity_.setIdentity();
            Eigen::Affine3f trans(identity_.slerp(ratio, relative_quat));
            trans.translation() = ratio * relative_tran;
            Eigen::Vector3f pt(point.x, point.y, point.z);
            Eigen::Vector3f comp_pt = pt;

            if(max_time - min_time > 1e-3){
                comp_pt = trans * pt;
                std::cout<<"trans the point"<<std::endl;
            }

//            std::cout<<"org pt "<<pt.transpose()<<std::endl;
//            std::cout<<"comp pt "<<comp_pt.transpose()<<std::endl;
//            std::cout<<"trans pose " <<trans.matrix()<<std::endl;

            if( std::isnan(comp_pt[0]) || std::isnan(comp_pt[1]) || std::isnan(comp_pt[2]) )
                continue;
            pcl::PointXYZ pc;
            pc.x = comp_pt[0];
            pc.y = comp_pt[1];
            pc.z = comp_pt[2];

            temp_pc->points.push_back(pc);
//            temp_pc->points[i].x = comp_pt[0];
//            temp_pc->points[i].y = comp_pt[1];
//            temp_pc->points[i].z = comp_pt[2];
        }

//        std::cout<<"org pc is "<<org_pc->size()<<std::endl;
//        std::cout<<"temp pc is "<<temp_pc->size()<<std::endl;

        pcl::NormalEstimationOMP<pcl::PointXYZ, PointXYZN> norm_est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());

        norm_est.setNumberOfThreads(8);
        norm_est.setInputCloud(temp_pc);
        norm_est.setSearchMethod(kdtree);
        norm_est.setKSearch(20);
        norm_est.compute(*comp_pc);

    }

}  // namespace lidar_align
