#ifndef __ICP_H_
#define __ICP_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>

#include <eigen3/Eigen/Core>

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/types.h>
#include <ceres/rotation.h>
#include <ceres/loss_function.h>

#include <iostream>
#include <string>

typedef pcl::PointXYZ Point;

class ICP {
public:
    struct Point2PointError_CeresAngleAxis {
        const Eigen::Vector3f &p_dst;
        const Eigen::Vector3f &p_src;

        Point2PointError_CeresAngleAxis(const Eigen::Vector3f src, const Eigen::Vector3f dst):p_src(src), p_dst(dst) {}

        static ceres::CostFunction *Create(const Eigen::Vector3f &src, const Eigen::Vector3f &dst) {
            return(new ceres::AutoDiffCostFunction<Point2PointError_CeresAngleAxis, 3, 6>(
                    new Point2PointError_CeresAngleAxis(src, dst)));
        }

        template<typename T>
        bool operator()(const T *const camera, T *residuals) const {
            T p[3] = {T(p_src[0]), T(p_src[1]), T(p_src[2])};
            ceres::AngleAxisRotatePoint(camera, p, p);

            // camera[3,4,5] are the translation.
            p[0] += camera[3];
            p[1] += camera[4];
            p[2] += camera[5];

            // The error is the difference between the predicted and observed position.
            residuals[0] = p[0] - T(p_dst[0]);
            residuals[1] = p[1] - T(p_dst[1]);
            residuals[2] = p[2] - T(p_dst[2]);

            return true;
        }
    };

    ICP(std::string source_pcl_file, std::string target_pcl_file);
    ~ICP();
    void setSourcePointCloud(pcl::PointCloud<Point> &pcl_source);
    void setTargetPointCloud(pcl::PointCloud<Point> &pcl_target);
    void runSVDMatch(void);
    void runOptimationMatch(void);
private:
    std::string source_pcl_file_;
    std::string target_pcl_file_;

    pcl::PointCloud<Point> pcl_source_;
    pcl::PointCloud<Point> pcl_target_;
    pcl::PointCloud<Point> pcl_source_no_nan_;
    pcl::PointCloud<Point> pcl_target_no_nan_;

    std::vector<Eigen::Vector3f> pcl_source_no_nan_vec_;
    std::vector<Eigen::Vector3f> pcl_target_no_nan_vec_;

    std::vector<Eigen::Vector3f> pcl_source_vec_;
    std::vector<Eigen::Vector3f> pcl_target_vec_;

    std::vector<Eigen::Vector3f> pcl_source_vec_rt_center_;
    std::vector<Eigen::Vector3f> pcl_target_vec_rt_center_;

    Eigen::Vector3f source_center_;
    Eigen::Vector3f target_center_;

    pcl::visualization::PCLVisualizer viewer;
};

#endif