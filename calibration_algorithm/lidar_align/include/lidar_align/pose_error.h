#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigen>
#include <string>
#include <vector>

struct HandEyeCostFunction {
    HandEyeCostFunction(const std::vector<Eigen::Affine3d> &src,
                        const std::vector<Eigen::Affine3d> &base)
            : src_poses(src), base_poses(base) {
        pose_size = src_poses.size();
    }

    HandEyeCostFunction(const Eigen::Affine3d &src, const Eigen::Affine3d &base) {
        src_poses.push_back(src);
        base_poses.push_back(base);
        pose_size = src_poses.size();
    }

    template<typename T>
    bool operator()(const T *x, const T *y, const T *z, const T *rotation,
                    T *residual) const {
        Eigen::Quaternion<T> quat(rotation[3], rotation[0], rotation[1],
                                  rotation[2]);
        Eigen::Transform<T, 3, Eigen::Affine> ext;
        ext.translation() = Eigen::Matrix<T, 3, 1>(*x, *y, *z);
        ext.linear() = quat.toRotationMatrix();
        T error = T(0);
        for (int i = 0; i < pose_size; ++i) {
            Eigen::Transform<T, 3, Eigen::Affine> pose1 = ext * src_poses[i].cast<T>();
            Eigen::Transform<T, 3, Eigen::Affine> pose2 = base_poses[i].cast<T>() * ext;
            Eigen::Transform<T, 3, Eigen::Affine> delta = pose1.inverse() * pose2;
            Eigen::Quaternion<T> delta_quat(delta.linear());
//            error = error + delta.translation().norm();
//            error = error + (delta_quat.w() - T(1.0)) * (delta_quat.w() - T(1.0));
            error = error + delta.translation()[0] * delta.translation()[0];
            error = error + delta.translation()[1] * delta.translation()[1];
            error = error + delta.translation()[2] * delta.translation()[2];
            error = error + delta_quat.x() * delta_quat.x();
            error = error + delta_quat.y() * delta_quat.y();
            error = error + delta_quat.z() * delta_quat.z();
        }

        *residual = ceres::sqrt(error);
        return true;
    }

    int pose_size;
    std::vector<Eigen::Affine3d> src_poses;
    std::vector<Eigen::Affine3d> base_poses;
};

