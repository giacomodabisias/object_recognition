#include "Icp.h"

ICP::ICP ()
  {
    if(!use_generalized_icp)
      icp_ = new pcl::IterativeClosestPoint<PointType, PointType>();
    else
      icp_ = new pcl::GeneralizedIterativeClosestPoint<PointType, PointType>();
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp_->setMaxCorrespondenceDistance (1); //5cm = 0.05
    // Set the maximum number of iterations (criterion 1)
    icp_->setMaximumIterations (20);
    // Set the transformation epsilon (criterion 2)
    icp_->setTransformationEpsilon (1e-6);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp_->setEuclideanFitnessEpsilon (1);

    fitness_score_ = 1;
    transformation_.Identity();
  }                 

void
ICP::Align ( pcl::PointCloud<PointType>::Ptr cloud_source,  pcl::PointCloud<PointType>::Ptr cloud_target)
{

  icp_->setInputSource (cloud_source);
  icp_->setInputTarget (cloud_target);

  // Perform the alignment
  icp_->align (*cloud_source);

  // Obtain the transformation that aligned cloud_source to cloud_source_registered
  transformation_ = icp_->getFinalTransformation ();
  fitness_score_ = icp_->getFitnessScore();
}

bool
ICP::HasConverged() 
{
  return icp_->hasConverged();
}
