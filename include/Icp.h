#ifndef ICP_H
#define ICP_H

#include "define.h"
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

class ICP{

public:
    Eigen::Matrix4f transformation_;
    Eigen::Matrix3f rotation_;
    Eigen::Vector3f traslation_;
    float fitness_score_;
    pcl::Registration<PointType, PointType> * icp_ = 0;

    ICP();

    void Align(pcl::PointCloud<PointType>::Ptr cloud_source, pcl::PointCloud<PointType>::Ptr cloud_target);
    bool HasConverged();
};


#endif