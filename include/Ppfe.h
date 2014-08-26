#ifndef PPFE_H
#define PPFE_H

#include "define.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/filters/extract_indices.h>

class Ppfe
{
  public:

    pcl::SACSegmentation<pcl::PointXYZ> seg_;
    pcl::ExtractIndices<pcl::PointXYZ> extract_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_xyz_;
    pcl::ModelCoefficients::Ptr coefficients_;
    pcl::PointIndices::Ptr inliers_;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_model_input_;
    pcl::PointCloud<pcl::PPFSignature>::Ptr cloud_model_ppf_;
    pcl::PPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf_registration_;
    pcl::PPFHashMapSearch::Ptr hashmap_search_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene_;
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
SubsampleAndCalculateNormals (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

#endif