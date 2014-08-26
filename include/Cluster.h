#ifndef CLUSTER_H
#define CLUSTER_H

#include "define.h"
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

class Hough
{
  public:
    ClusterType cluster_;
    pcl::PointCloud<RFType>::Ptr model_rf_;
    pcl::PointCloud<RFType>::Ptr scene_rf_;
    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est_;
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer_;
    bool created_;
    pcl::PointCloud<PointType>::Ptr model_;
    pcl::PointCloud<PointType>::Ptr model_keypoints_;
    pcl::PointCloud<NormalType>::Ptr model_normals_;

    Hough (pcl::PointCloud<PointType>::Ptr model, pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<NormalType>::Ptr model_normals);

    ClusterType
    GetClusters (pcl::PointCloud<PointType>::Ptr scene, pcl::PointCloud<PointType>::Ptr scene_keypoints,
        pcl::PointCloud<NormalType>::Ptr scene_normals, pcl::CorrespondencesPtr model_scene_corrs);

};

class GCG
{
  public:
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer_;
    ClusterType cluster_;

    GCG ();

    ClusterType
    GetClusters (pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<PointType>::Ptr scene_keypoints, pcl::CorrespondencesPtr model_scene_corrs);
};

#endif