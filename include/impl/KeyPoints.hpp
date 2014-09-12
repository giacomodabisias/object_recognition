#ifndef KEYPOINTS_HPP
#define KEYPOINTS_HPP

#include <pcl/sample_consensus/ransac.h>
#include "define.h"

template<class T>
class Ransac
{
  public:
    std::vector<int> cloud_inliers_;

    void
    GetKeypoints (pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints)
    {

      typename T::Ptr cloud_plane (new T (cloud));

      pcl::RandomSampleConsensus < pcl::PointXYZRGB > model_ransac (cloud_plane);
      model_ransac.computeModel ();
      model_ransac.getInliers (cloud_inliers_);

      pcl::copyPointCloud < pcl::PointXYZRGB > (*cloud, cloud_inliers_, *cloud_keypoints);
    }
};


#endif