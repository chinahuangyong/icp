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

#include <iostream>
#include <string>

typedef pcl::PointXYZ Point;

class ICP {
public:
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

    std::vector<Eigen::Vector3f> pcl_source_vec_;
    std::vector<Eigen::Vector3f> pcl_target_vec_;

    std::vector<Eigen::Vector3f> pcl_source_vec_rt_center_;
    std::vector<Eigen::Vector3f> pcl_target_vec_rt_center_;

    Eigen::Vector3f source_center_;
    Eigen::Vector3f target_center_;

    pcl::visualization::PCLVisualizer viewer;
};

#endif