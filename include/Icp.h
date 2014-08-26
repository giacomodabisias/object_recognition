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

    virtual void Align(pcl::PointCloud<PointType>::Ptr cloud_source, pcl::PointCloud<PointType>::Ptr cloud_target) = 0;
};


class ICPRegistration : public ICP
{
  public:
    pcl::IterativeClosestPoint<PointType, PointType> icp_;

    
    ICPRegistration ();
                   
    void
    Align ( pcl::PointCloud<PointType>::Ptr cloud_source,  pcl::PointCloud<PointType>::Ptr cloud_target);
   
};

class GeneralizedICPRegistration : public ICP
{
  public:
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp_;

    
    GeneralizedICPRegistration ();
                  
    void
    Align ( pcl::PointCloud<PointType>::Ptr cloud_source,  pcl::PointCloud<PointType>::Ptr cloud_target);
 
};

#endif