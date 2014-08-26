#ifndef KEYPOINTS_H
#define KEYPOINTS_H

#include "define.h"
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/sample_consensus/sac_model_sphere.h>




class Narf
{
  public:
    pcl::PointCloud<int> cloud_keypoint_indices_;
    Eigen::Affine3f cloud_sensor_pose_;
    bool rotation_invariant_;
    pcl::RangeImageBorderExtractor range_image_border_extractor_;
    pcl::NarfKeypoint narf_keypoint_detector_;

    Narf ();

    void
    GetKeypoints (pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints);
};

class Sift
{
  public:
    pcl::PointCloud<pcl::PointWithScale> cloud_result_;
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_;

    Sift ();

    void
    GetKeypoints (pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints);
};

class Harris
{
  public:
    pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI>* harris3D_;

    Harris ();

    void
    GetKeypoints (pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints);
};

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

class Uniform
{
  public:
    pcl::UniformSampling<PointType> uniform_sampling_;
    pcl::PointCloud<int> sampled_indices_;
    float cloud_ss_ = 0;

    void
    SetSamplingSize (float sampling_size);

    void
    GetKeypoints (pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints);

};

#endif