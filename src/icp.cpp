#include "../include/icp.h"

#include <glog/logging.h>

ICP::ICP(std::string source_pcl_file, std::string target_pcl_file) {
    source_pcl_file_ = source_pcl_file;
    target_pcl_file_ = target_pcl_file;

    if(pcl::io::loadPCDFile<Point>(source_pcl_file_, pcl_source_) != 0) {
        LOG(INFO) << "read source pcd file failed";
        return;
    } else {
        LOG(INFO) << "read source pcd file success";
    }

    if(pcl::io::loadPCDFile<Point>(target_pcl_file_, pcl_target_) != 0) {
        LOG(INFO) << "read target pcd file failed";
        return;
    } else {
        LOG(INFO) << "read target pcd file success";
    }
}

ICP::~ICP() {

}

void ICP::setSourcePointCloud(pcl::PointCloud<Point> &pcl_source) {
    pcl_source_ = pcl_source;
    // Eigen::Vector3f vec(0, 0, 0);
    // for(auto& point: pcl_source.points) {
    //     vec << point.x, point.y, point.z;
    //     pcl_source_vec_.push_back(vec);
    // }
}

void ICP::setTargetPointCloud(pcl::PointCloud<Point> &pcl_target) {
    pcl_target_ = pcl_target;
}

// 使用SVD进行ICP匹配
void ICP::runSVDMatch() {
    //[1] 取出无效点
    std::vector<int> index;

    pcl::removeNaNFromPointCloud(pcl_source_, pcl_source_no_nan_, index);
    pcl::removeNaNFromPointCloud(pcl_target_, pcl_target_no_nan_, index);

    source_center_.setZero();
    target_center_.setZero();

    //[2] 将点云的数据转到Eigen中做进行处理
    for(auto point : pcl_source_no_nan_.points) {
        Eigen::Vector3f data;
        data << point.x, point.y, point.z;
        source_center_ = source_center_ + data;
        pcl_source_vec_.push_back(data);
    }
    source_center_ = source_center_ / pcl_source_no_nan_.size();

    for(auto point : pcl_target_no_nan_.points) {
        Eigen::Vector3f data;
        data << point.x, point.y, point.z;
        target_center_ = target_center_ + data;
        pcl_target_vec_.push_back(data);
    }
    target_center_ = target_center_ / pcl_target_no_nan_.size();

    // [3]求每个点想对于中心点的坐标
    for(auto &point : pcl_source_vec_) {
        pcl_source_vec_rt_center_.push_back(point - source_center_);
    }

    for(auto& point : pcl_target_vec_) {
        pcl_target_vec_rt_center_.push_back(point - target_center_);
    }

    //[4] 求H矩阵
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> source_pcl_matrix;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> target_pcl_matrix;

    source_pcl_matrix.resize(3, pcl_source_vec_rt_center_.size());
    target_pcl_matrix.resize(3, pcl_target_vec_rt_center_.size());

    for(int i=0; i<pcl_source_vec_rt_center_.size(); i++) {
        source_pcl_matrix.col(i) = pcl_source_vec_rt_center_.at(i);
    }

    for(int i=0; i<pcl_target_vec_rt_center_.size(); i++) {
        target_pcl_matrix.col(i) = pcl_target_vec_rt_center_.at(i);
    }

    //[5] 对H矩阵进行SVD分解
    auto H = source_pcl_matrix * target_pcl_matrix.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3f R;
    auto V = svd.matrixV();
    auto U = svd.matrixU();

    //[6] 获得旋转矩阵
    R = V * U.transpose();
    std::cout << "R: " << std::endl << R << std::endl;

    //[7] 获得平移矩阵
    Eigen::Vector3f transl = target_center_ - R*source_center_;
    std::cout << "transl: " << std::endl << transl << std::endl;
}


void ICP::runOptimationMatch() {

}