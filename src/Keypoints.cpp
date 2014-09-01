#include "Keypoints.hpp"

Narf::Narf () :
    rotation_invariant_ (true), cloud_sensor_pose_ (Eigen::Affine3f::Identity ())
{
  narf_keypoint_detector_.setRangeImageBorderExtractor (&range_image_border_extractor_);
  narf_keypoint_detector_.getParameters ().support_size = support_size;

}

void
Narf::GetKeypoints (pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints)
{

  boost::shared_ptr < pcl::RangeImage > cloud_range_image_ptr_ (new pcl::RangeImage);

  cloud_sensor_pose_ = Eigen::Affine3f (Eigen::Translation3f (cloud->sensor_origin_[0], cloud->sensor_origin_[1], cloud->sensor_origin_[2])) * Eigen::Affine3f (cloud->sensor_orientation_);

  pcl::RangeImage& cloud_range_image_ = *cloud_range_image_ptr_;

  narf_keypoint_detector_.setRangeImage (&cloud_range_image_);

  cloud_range_image_.createFromPointCloud (*cloud, pcl::deg2rad (0.5f), pcl::deg2rad (360.0f), pcl::deg2rad (180.0f), cloud_sensor_pose_, pcl::RangeImage::CAMERA_FRAME, 0.0, 0.0f, 1);

  cloud_range_image_.setUnseenToMaxRange ();

  narf_keypoint_detector_.compute (cloud_keypoint_indices_);

  cloud_keypoints->points.resize (cloud_keypoint_indices_.points.size ());

  #pragma omp parallel for
  for (size_t i = 0; i < cloud_keypoint_indices_.points.size (); ++i)
    cloud_keypoints->points[i].getVector3fMap () = cloud_range_image_.points[cloud_keypoint_indices_.points[i]].getVector3fMap ();
}


Sift::Sift () :
    tree_ (new pcl::search::KdTree<pcl::PointXYZRGB> ())
{
  sift_.setSearchMethod (tree_);
  sift_.setScales (min_scale, n_octaves, n_scales_per_octave);
  sift_.setMinimumContrast (min_contrast);
}

void
Sift::GetKeypoints (pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints)
{

  sift_.setInputCloud (cloud);
  sift_.compute (cloud_result_);
  copyPointCloud (cloud_result_, *cloud_keypoints);
}


Harris::Harris () :
    harris3D_ (new pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI> (pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI>::HARRIS))
{
  harris3D_->setNonMaxSupression (true);
  harris3D_->setRadius (0.03);
  harris3D_->setRadiusSearch (0.03);
  switch (harris_type)
  {
    default:
      harris3D_->setMethod (pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::HARRIS);
      break;
    case 2:
      harris3D_->setMethod (pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::NOBLE);
      break;
    case 3:
      harris3D_->setMethod (pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE);
      break;
    case 4:
      harris3D_->setMethod (pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::TOMASI);
      break;
    case 5:
      harris3D_->setMethod (pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::CURVATURE);
      break;
  }
}

void
Harris::GetKeypoints (pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints)
{
  harris3D_->setInputCloud (cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp (new pcl::PointCloud<pcl::PointXYZI>);
  harris3D_->compute (*keypoints_temp);
  copyPointCloud (*keypoints_temp, *cloud_keypoints);
}

void
Uniform::SetSamplingSize (float sampling_size)
{
  cloud_ss_ = sampling_size;
}

void
Uniform::GetKeypoints (pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints)
{
  if (cloud_ss_ != 0)
  {
    uniform_sampling_.setInputCloud (cloud);
    uniform_sampling_.setRadiusSearch (cloud_ss_);
    uniform_sampling_.compute (sampled_indices_);
    pcl::copyPointCloud (*cloud, sampled_indices_.points, *cloud_keypoints);
  }
  else
    std::cout << "no sampling size inserted" << std::endl;
}

   