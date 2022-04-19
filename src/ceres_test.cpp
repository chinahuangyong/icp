#include <ceres/ceres.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/local_parameterization.h>
#include <ceres/types.h>

#include <glog/logging.h>

#include <iostream>
#include <vector>

// 自动求导
// struct CurveFilterCost {
//     double x_, y_;
//     CurveFilterCost(double x, double y):x_(x), y_(y) {}
//
//     template<typename T>
//     bool operator()(const T* const abc, T* residual) const{
//         residual[0] = T(y_) - ceres::exp(abc[0]*T(x_)* T(x_) +
//                 abc[1]*T(x_) + abc[2]);
//         return true;
//     }
// };

//解析求导
// class CurveFilterCost : public ceres::SizedCostFunction<1, 3>{
// public:
//     const double x_, y_;
//
//     CurveFilterCost(const double x, const double y):x_(x), y_(y) {}
//     virtual ~CurveFilterCost(){}
//
//     virtual bool Evaluate(double const *const *parameters,
//                           double *residuals,
//                           double **jacobians) const {
//         const double a = parameters[0][0];
//         const double b = parameters[0][1];
//         const double c = parameters[0][2];
//
//         residuals[0] = y_ - ceres::exp(a*x_*x_ +
//                 b*x_+ c);
//
//         if(!jacobians) {
//             return true;
//         }
//         double *jacos = jacobians[0];
//         if(!jacos) {
//             return true;
//         }
//
//         jacos[0] = -1*ceres::exp(a*x_*x_ + b*x_ + c)*(x_)*(x_);
//         jacos[1] = -1*ceres::exp(a*x_*x_ + b*x_ + c)*(x_);
//         jacos[2] = -1*ceres::exp(a*x_*x_ + b*x_ + c);
//
//         return true;
//     }
//
// };

//数值求导
struct CurveFilterCost {
public:
    const double x_, y_;
    CurveFilterCost(double x, double y): x_(x), y_(y){}
    bool operator()(const double *const abc, double *residual) const {
        residual[0] = y_ - ceres::exp(abc[0]*x_*x_ + abc[1]*x_+ abc[2]);
        return true;
    }
};

int main(int argc, char** argv) {
    double a = 1.0, b=2.0, c=3.0;

    int N = 100;
    double w_sigma = 1.0;
    double abc[3] = {0, 0, 0};

    std::vector<double> x_data, y_data;

    for(int i=0; i<N; i++) {
        double x = (double )i/100;
        x_data.push_back(x);
        y_data.push_back(exp(a*x*x + b*x + c));
    }

    ceres::Problem problem;
    // for(int i=0; i<N; i++) {
    //     problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CurveFilterCost, 1, 3>(
    //             new CurveFilterCost(x_data.at(i), y_data.at(i))),
    //             nullptr,
    //             abc);
    // }

    // for(int i=0; i<N; i++) {
    //     ceres::CostFunction *cost_function = new CurveFilterCost(x_data.at(i), y_data.at(i));
    //     problem.AddResidualBlock(cost_function, nullptr, abc);
    // }

    for(int i=0; i<N; i++) {
        ceres::CostFunction *cost_function = new ceres::NumericDiffCostFunction<CurveFilterCost, ceres::CENTRAL, 1, 3>(
                new CurveFilterCost(x_data.at(i), y_data.at(i)));
        problem.AddResidualBlock(cost_function, nullptr, abc);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << summary.FullReport();

    std::cout << "a: " << abc[0] <<" b: " << abc[1] << " c: " << abc[2] << std::endl;

}