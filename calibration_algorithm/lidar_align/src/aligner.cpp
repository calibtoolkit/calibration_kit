#include "lidar_align/aligner.h"
#include "lidar_align/eigen_quaternion_parameterization.h"
#include "lidar_align/pose_error.h"
#include "lidar_align/sensors.h"
#include "lidar_align/transform.h"

namespace lidar_align {

    Aligner::Aligner(KeyValues conf_data) {
        Eigen::Quaternionf quat(std::stof(conf_data["inital_guess_rw"]),
                                std::stof(conf_data["inital_guess_rx"]),
                                std::stof(conf_data["inital_guess_ry"]),
                                std::stof(conf_data["inital_guess_rz"]));
        Eigen::Vector3f vec(std::stof(conf_data["inital_guess_tx"]),
                            std::stof(conf_data["inital_guess_ty"]),
                            std::stof(conf_data["inital_guess_tz"]));

        config_.ext_.rotation_ = quat;
        config_.ext_.translation_ = vec;

        config_.min_pair_dist = std::stof(conf_data["min_pair_dist"]);
        config_.min_pair_yaw = std::stof(conf_data["min_pair_yaw"]);

        config_.max_icp_iteration = std::stoi(conf_data["max_icp_iteration"]);
        config_.max_icp_corres_dist = std::stof(conf_data["max_icp_corres_dist"]);
        config_.min_icp_euclid_eps = std::stof(conf_data["min_icp_euclid_eps"]);
        config_.min_icp_transform_eps = std::stof(conf_data["min_icp_transform_eps"]);

        config_.max_update_iteration = std::stoi(conf_data["max_update_iteration"]);
        config_.update_termination_dist_eps =
                std::stof(conf_data["update_termination_dist_eps"]);
        config_.update_termination_angle_eps =
                std::stof(conf_data["update_termination_angle_eps"]);
        config_.fix_extrinsic_z = string_to_bool(conf_data["fix_extrinsic_z"]);
        config_.default_optimization_iter =
                std::stof(conf_data["default_optimization_iter"]);
    };

    void Aligner::align(Odom odom, std::vector<std::string> pcd_dirs,
                        std::vector<double> pcd_timestamps) {
        // std::vector<std::pair<int, int>> pairs;
        int odom_size = odom.getOdomTransformSize();
        std::cout<<"odom size is "<<odom_size<<std::endl;

        std::vector<Transform> lidar_delta;
        std::vector<Transform> odom_delta;

        std::vector<std::pair<double, double> > lidar_frame_time_pair;
        std::vector<std::pair<Transform, Transform>> lidar_frame_pose_pair;

        for (int i = 0; i < pcd_dirs.size(); i++) {
            LoaderPointcloudPtr org_pc(new LoaderPointcloud);
            pcl::io::loadPCDFile(pcd_dirs[i], *org_pc);
            std::vector<int> indices;
//            pcl::removeNaNFromPointCloud<PointXYZIT>(*org_pc, *org_pc, indices);

            double min_time = 1e15;
            double max_time = -1e15;

            for (int index = 0; index < org_pc->size(); index++) {
                min_time = org_pc->points[i].timestamp < min_time ? org_pc->points[i].timestamp : min_time;
                max_time = org_pc->points[i].timestamp > max_time ? org_pc->points[i].timestamp : max_time;
            }


//            std::cout<<"getOdomTransform aligner 64"<<std::endl;
            lidar_frame_time_pair.emplace_back(min_time, max_time);
            Transform start_pose = odom.getOdomTransform(min_time);
            Transform end_pose = odom.getOdomTransform(max_time);
//            std::cout.precision(20);
//            std::cout<<"start timestamp is "<<min_time<<"   "<<"end timestamp "<<max_time<<std::endl;
//            std::cout<<"start pose is "<<std::endl<<start_pose.matrix()<<std::endl<<"end pose is "<<std::endl<<end_pose.matrix()<<std::endl;

            lidar_frame_pose_pair.emplace_back(start_pose, end_pose);

            original_point_clouds_.push_back(org_pc);
            PointCloudXYZNPtr comp_pc(new PointCloudXYZN);
            pcl::copyPointCloud(*org_pc, *comp_pc);
            compensated_point_clouds_.push_back(comp_pc);
        }

        std::cout<<"load the pcd "<<std::endl;
        std::cout<<"original_point_clouds_ size is "<<original_point_clouds_.size()<<std::endl;

        for(int iter = 0; iter < 50; iter ++){

            ceres::Problem problem;
            ceres::LossFunction *loss_func_ptr = new ceres::CauchyLoss(0.2);
            ceres::Solver::Summary summary;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 500;

            Eigen::Quaternionf quat = config_.ext_.rotation_;
            double translation[3] = {config_.ext_.translation_[0],
                                     config_.ext_.translation_[1],
                                     config_.ext_.translation_[2]};
            double quaternion[4] = {quat.x(), quat.y(), quat.z(), quat.w()};
            omp_lock_t lock;
            omp_init_lock(&lock);
#pragma omp parallel for // NOLINT(openmp-use-default-none)
            for(int cloud_index = 0; cloud_index < original_point_clouds_.size(); cloud_index++){
                auto org_pc = original_point_clouds_[cloud_index];
                auto comp_pc = compensated_point_clouds_[cloud_index];
                auto time = lidar_frame_time_pair[cloud_index];
                auto pose = lidar_frame_pose_pair[cloud_index];
                LidarMotionCompensation(config_.ext_, org_pc, comp_pc, time.first, time.second, pose.first, pose.second);
            }
            omp_destroy_lock(&lock);

            std::vector<std::pair<int, int>> match_pair;

            int last_source_id = 0;
            for (int i = 0; i < pcd_timestamps.size(); i++) {

                Transform last_src_pos = odom.getOdomTransform(pcd_timestamps[last_source_id]);
                Transform cur_src_pos = odom.getOdomTransform(pcd_timestamps[i]);
                Transform delta_src_pos = cur_src_pos.inverse() * last_src_pos;

                if(std::abs(delta_src_pos.rotation_.matrix().eulerAngles(2, 1, 0)[0] * 180 / 3.1415926) < config_.min_pair_yaw &&
                 delta_src_pos.translation_.norm() < config_.min_pair_dist)
                {
                    continue;
                }
                last_source_id = i;
                int last_id = i;
                for (int j = i + 1; j < pcd_timestamps.size(); j++) {
                    Transform src_pos = odom.getOdomTransform(pcd_timestamps[last_id]);
                    Transform base_pos = odom.getOdomTransform(pcd_timestamps[j]);
                    Transform delta_pos = base_pos.inverse() * src_pos;

                    Transform delta_src_pos = base_pos.inverse() * cur_src_pos;

                    Eigen::Vector3f euler_angle = delta_pos.rotation_.matrix().eulerAngles(2, 1, 0);
                    double yaw = std::abs(euler_angle[0] * 180 / 3.1415926);
                    double dist = delta_pos.translation_.norm();

                    if (yaw > config_.min_pair_yaw) {
                        if (dist > config_.min_pair_dist && delta_src_pos.translation_.norm() < 10) {
                            match_pair.emplace_back(i, j);
                            last_id = j;
                        }
                    }
                }


            }

            std::cout<<"get pair size is "<<match_pair.size()<<std::endl;
            omp_init_lock(&lock);
#pragma omp parallel for // NOLINT(openmp-use-default-none)
            for (int i = 0; i < match_pair.size(); i++) {
                auto idx_pair = match_pair[i];
                Transform src_pos = odom.getOdomTransform(pcd_timestamps[idx_pair.first]);
                Transform base_pos = odom.getOdomTransform(pcd_timestamps[idx_pair.second]);
                Transform delta_pos = base_pos.inverse() * src_pos;
                IcpState icp_state;
                icp_state.trans = delta_pos;
                icp_match(compensated_point_clouds_[idx_pair.first], compensated_point_clouds_[idx_pair.second], icp_state);

                Transform delta_ = config_.ext_.inverse() * delta_pos * config_.ext_;

                omp_set_lock(&lock);
                if (icp_state.valid) {
                    lidar_delta.push_back(icp_state.trans);
                    odom_delta.push_back(delta_pos);

                    std::cout<<"icp trans "<<icp_state.trans.translation_.transpose()<<std::endl;
                    std::cout<<"delta trans "<<delta_.translation_.transpose()<<std::endl;
                    std::cout<<"icp rot "<<icp_state.trans.rotation_.coeffs().transpose()<<std::endl;
                    std::cout<<"delta rot "<<delta_.rotation_.coeffs().transpose()<<std::endl;
                }
                omp_unset_lock(&lock);

            }
            omp_destroy_lock(&lock);


            for (size_t i = 0; i < lidar_delta.size(); ++i) {
                Eigen::Affine3d lidar_transform;
                Eigen::Affine3d odom_transform;

                lidar_transform.translation() = lidar_delta[i].translation_.cast<double>();
                lidar_transform.linear() =
                        lidar_delta[i].rotation_.toRotationMatrix().cast<double>();

                odom_transform.translation() = odom_delta[i].translation_.cast<double>();
                odom_transform.linear() =
                        odom_delta[i].rotation_.toRotationMatrix().cast<double>();

                ceres::CostFunction *cost_func =
                        new ceres::AutoDiffCostFunction<HandEyeCostFunction, 1, 1, 1, 1,
                        4>(new HandEyeCostFunction(
                                lidar_transform, odom_transform));
                problem.AddResidualBlock(cost_func, loss_func_ptr, translation,
                                         translation + 1, translation + 2, quaternion);
            }

            ceres::LocalParameterization *local_para_ptr = new EigenQuaternionParameterization;
            problem.SetParameterization(quaternion, local_para_ptr);

            if (fix_extrinsic_z_) {
                problem.SetParameterBlockConstant(translation + 2);
            }

            ceres::Solve(options, &problem, &summary);
            Eigen::Quaternionf quat_result(quaternion[3], quaternion[0], quaternion[1],
                                           quaternion[2]);
            Eigen::Vector3f tras_result = Eigen::Vector3f(translation[0], translation[1], translation[2]);
            Transform pose_result;
            pose_result.rotation_ = quat_result;
            pose_result.translation_ = tras_result;

            Transform delta_result = config_.ext_.inverse() * pose_result;

            std::cout<<"the iter config extrinsic is "<<std::endl;
            std::cout<<"old eular is "<<config_.ext_.rotation_.matrix().eulerAngles(2, 0, 1).transpose()  * 180 / 3.1415926 <<std::endl;
            std::cout<<"old translation is "<<config_.ext_.translation_.transpose()<<std::endl;

            std::cout<<"pose_result eular is "<<pose_result.rotation_.matrix().eulerAngles(2, 0, 1).transpose() * 180 / 3.1415926<<std::endl;
            std::cout<<"pose_result translation is "<<pose_result.translation_.transpose()<<std::endl;

            std::cout<<"delta_result.translation_.norm() "<<delta_result.translation_.norm()<<std::endl;
            std::cout<<"delta_result.rotation_.w() "<<delta_result.rotation_.w()<<std::endl;

            if(delta_result.translation_.norm() < 0.05 && delta_result.rotation_.w() - 1.0 < 0.05){
                break;
            }

            config_.ext_ = pose_result;

            std::cout<<"the iter config extrinsic is "<<std::endl;
            std::cout<<"eular is "<<config_.ext_.rotation_.matrix().eulerAngles(2, 0, 1).transpose()<<std::endl;
            std::cout<<"translation is "<<config_.ext_.translation_.transpose()<<std::endl;
        }


    }

    void Aligner::icp_match(const PointCloudXYZNPtr &src,
                            const PointCloudXYZNPtr &base, IcpState &icp_state) {
        pcl::IterativeClosestPointWithNormals<PointXYZN, PointXYZN, float> icp;
        icp.setInputSource(src);
        icp.setInputTarget(base);

        icp.setMaxCorrespondenceDistance(config_.max_icp_corres_dist);
        icp.setMaximumIterations(config_.max_icp_iteration);
        icp.setTransformationEpsilon(config_.min_icp_transform_eps);
        icp.setEuclideanFitnessEpsilon(config_.min_icp_euclid_eps);

        PointCloudXYZNPtr src_registered(new PointCloudXYZN);

        Transform init = config_.ext_.inverse() * icp_state.trans;
        init = init * config_.ext_;

        Eigen::Matrix4f init_pose;
        init_pose.block(0, 0, 3, 3) = init.rotation_.toRotationMatrix();
        init_pose.block(0, 3, 3, 1) = init.translation_;

        icp.align(*src_registered, init_pose);
        if (icp.hasConverged()) {
            init_pose = icp.getFinalTransformation().cast<float>();
            icp_state.valid = true;
            icp_state.error = icp.getFitnessScore();
            Eigen::Matrix3f temp = init_pose.block(0, 0, 3, 3);
            icp_state.trans.rotation_ = Eigen::Quaternionf(temp);
            icp_state.trans.translation_ = init_pose.block(0, 3, 3, 1);
        } else {
            icp_state.valid = false;
            std::cout << "align failed" << std::endl;
        }
    }

}  // namespace lidar_align
