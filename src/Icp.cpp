#include "Icp.h"

ICPRegistration::ICPRegistration ()
  {
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp_.setMaxCorrespondenceDistance (0.05);
    // Set the maximum number of iterations (criterion 1)
    icp_.setMaximumIterations (20);
    // Set the transformation epsilon (criterion 2)
    icp_.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp_.setEuclideanFitnessEpsilon (1);

    fitness_score_ = 1;

    transformation_.Identity();
  }                 

void
ICPRegistration::Align ( pcl::PointCloud<PointType>::Ptr cloud_source,  pcl::PointCloud<PointType>::Ptr cloud_target)
  {

    pcl::PointCloud<PointType>::Ptr cloud_source_registered (new  pcl::PointCloud<PointType> ());

    icp_.setInputSource (cloud_source);
    icp_.setInputTarget (cloud_target);

    // Perform the alignment
    icp_.align (*cloud_source);

    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    transformation_ = icp_.getFinalTransformation ();
    fitness_score_ = icp_.getFitnessScore();
  }

GeneralizedICPRegistration::GeneralizedICPRegistration ()
{
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp_.setMaxCorrespondenceDistance (0.05);
  // Set the maximum number of iterations (criterion 1)
  icp_.setMaximumIterations (20);//20
  // Set the transformation epsilon (criterion 2)
  icp_.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp_.setEuclideanFitnessEpsilon (1);

  fitness_score_ = 1;

  transformation_.Identity();
}                 

void
GeneralizedICPRegistration::Align ( pcl::PointCloud<PointType>::Ptr cloud_source,  pcl::PointCloud<PointType>::Ptr cloud_target)
{

   pcl::PointCloud<PointType>::Ptr cloud_source_registered (new  pcl::PointCloud<PointType> ());

  icp_.setInputSource (cloud_source);
  icp_.setInputTarget (cloud_target);

  // Perform the alignment
  icp_.align (*cloud_source);

  // Obtain the transformation that aligned cloud_source to cloud_source_registered
  transformation_ = icp_.getFinalTransformation ();
  fitness_score_ = icp_.getFitnessScore();
}
