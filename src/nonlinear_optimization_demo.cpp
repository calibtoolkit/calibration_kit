#include <Eigen/Core>
#include <ceres/ceres.h>
#include <iostream>
#include <Eigen/LU>

void residual(double x, double y, Eigen::Vector2d &residuals){
    residuals[0] = 3 * std::pow(x, 4) + 4 * std::pow(y, 3) + 5 * std::pow(x, 2) + 6 * y - 15;
    residuals[1] = 6 * std::pow(y, 4) + 5 * std::pow(x ,3) + 4 * std::pow(y, 2) + 3 * x - 12;
}

void jacobian(double x, double y, Eigen::Matrix2d &Jacob){
    Jacob(0,0) = 12 * std::pow(x, 3) + 10 * x;
    Jacob(1,0) = 12 * std::pow(y, 2) + 6;
    Jacob(0,1) = 15 * std::pow(x, 2) + 3;
    Jacob(1,1) = 24 * std::pow(y, 3) + 8 * y;
}

struct residual_ceres
{
    residual_ceres() {}

    template <typename T>
    bool operator()(const T* const x , const T* const y, T* e) const {
        e[0] = T(3) * ceres::pow(x[0], T(4)) + T(4) * ceres::pow(y[0], T(3)) + T(5) * ceres::pow(x[0], T(2)) + T(6) * y[0] - T(15);
        e[1] = T(6) * ceres::pow(y[0], T(4)) + T(5) * ceres::pow(x[0] ,T(3)) + T(4) * ceres::pow(y[0], T(2)) + T(3) * x[0] - T(12);
        return true;
    }

};

void nonlinear_opt_demo(){

    double x = 0.001;
    double y = 0.001;

    Eigen::Vector2d residuals;
    residuals.setZero();

    residual(x, y, residuals);

    std::cout<<"the init residual is "<<residuals.norm()<<std::endl;

    int iter = 0;

    while(residuals.norm() > 0.1 && iter < 50){
        iter++;
        Eigen::Matrix2d jacob;
        jacobian(x, y, jacob);



        Eigen::Matrix2d hessian = jacob.transpose() * jacob;
        Eigen::Vector2d b = -(jacob.transpose() * residuals);
        // Eigen::Vector2d update_ = hessian.ldlt().solve(b) ;
        Eigen::Vector2d update_ = hessian.inverse() * b;

        x += update_(0);
        y += update_(1);

        residual(x, y, residuals);

        std::cout<<"the residual is "<<residuals.norm()<<std::endl;
    }

    std::cout<<"the result is "<<x<<"   "<<y<<std::endl;

    x = 0; y = 0;

    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<residual_ceres, 2, 1, 1>(new residual_ceres());
    // residual_factor *cost_function = new residual_factor();

    ceres::Problem problem;
    problem.AddParameterBlock(&x, 1);
    problem.AddParameterBlock(&y, 1);

    ceres::LossFunction* loss_func_ptr = new ceres::CauchyLoss(0.2);
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;

    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;

    problem.AddResidualBlock(cost_function, nullptr, &x, &y);

    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;


    residual(x, y, residuals);

    std::cout<<"the final residual is "<<residuals.norm()<<std::endl;

    std::cout<<"x is "<<x<<" y is "<<y<<std::endl;
}
