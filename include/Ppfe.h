#ifndef OBJECT_RECOGNITION_PPFE_H
#define OBJECT_RECOGNITION_PPFE_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/filters/extract_indices.h>

#include "define.h"

class Ppfe
{
  public:

    pcl::SACSegmentation<XYZType> seg_;
    pcl::ExtractIndices<XYZType> extract_;
    pcl::PointCloud<XYZType>::Ptr model_xyz_;
    pcl::ModelCoefficients::Ptr coefficients_;
    pcl::PointIndices::Ptr inliers_;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_model_input_;
    pcl::PointCloud<pcl::PPFSignature>::Ptr cloud_model_ppf_;
    pcl::PPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf_registration_;
    pcl::PPFHashMapSearch::Ptr hashmap_search_;
    pcl::PointCloud<XYZType>::Ptr cloud_scene_;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_scene_input_;
    pcl::PointCloud<pcl::PointNormal> cloud_output_subsampled_;
    Eigen::Matrix4f mat_;
    ClusterType cluster_;
    unsigned nr_points_;

    Ppfe (pcl::PointCloud<PointType>::Ptr model);

    ClusterType
    GetCluster (pcl::PointCloud<PointType>::Ptr scene);

    pcl::PointCloud<PointType>::Ptr
    GetModelKeypoints ();

    pcl::PointCloud<PointType>::Ptr
    GetSceneKeypoints ();

};

pcl::PointCloud<pcl::PointNormal>::Ptr
SubsampleAndCalculateNormals (pcl::PointCloud<XYZType>::Ptr cloud);

#endif