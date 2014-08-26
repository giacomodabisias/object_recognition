#include "Cluster.h"



Hough::Hough (pcl::PointCloud<PointType>::Ptr model, pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<NormalType>::Ptr model_normals) :
model_rf_ (new pcl::PointCloud<RFType> ()), scene_rf_ (new pcl::PointCloud<RFType> ()), created_ (false), model_(model), model_keypoints_(model_keypoints),
model_normals_(model_normals)
{
  rf_est_.setFindHoles (true);
  rf_est_.setRadiusSearch (rf_rad);
  clusterer_.setHoughBinSize (cg_size);
  clusterer_.setHoughThreshold (cg_thresh);
  clusterer_.setUseInterpolation (true);
  clusterer_.setUseDistanceWeight (false);
  rf_est_.setInputCloud (model_keypoints_);
  rf_est_.setInputNormals (model_normals_);
  rf_est_.setSearchSurface (model_);
  rf_est_.compute (*model_rf_);
}

ClusterType
Hough::GetClusters ( pcl::PointCloud<PointType>::Ptr scene, pcl::PointCloud<PointType>::Ptr scene_keypoints,
    pcl::PointCloud<NormalType>::Ptr scene_normals, pcl::CorrespondencesPtr model_scene_corrs)
{

  clusterer_.setHoughBinSize (cg_size);
  clusterer_.setHoughThreshold (cg_thresh);

  //  Compute (Keypoints) Reference Frames only for Hough
  rf_est_.setInputCloud (scene_keypoints);
  rf_est_.setInputNormals (scene_normals);
  rf_est_.setSearchSurface (scene);
  rf_est_.compute (*scene_rf_);


  //  Clustering
  if (!created_)
  {
    clusterer_.setInputCloud (model_keypoints_);
    clusterer_.setInputRf (model_rf_);
    created_ = true;
  }

  clusterer_.setSceneCloud (scene_keypoints);
  clusterer_.setSceneRf (scene_rf_);
  clusterer_.setModelSceneCorrespondences (model_scene_corrs);

  //std::cout << "prepared Hough for clustering" <<std::endl;

  //clusterer_.cluster_ (clustered_corrs);
  clusterer_.recognize (std::get < 0 > (cluster_), std::get < 1 > (cluster_));
  return (cluster_);
}


GCG::GCG ()
{
  gc_clusterer_.setGCSize (cg_size);
  gc_clusterer_.setGCThreshold (cg_thresh);
}

ClusterType
GCG::GetClusters (pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<PointType>::Ptr scene_keypoints, pcl::CorrespondencesPtr model_scene_corrs)
{

  gc_clusterer_.setInputCloud (model_keypoints);
  gc_clusterer_.setSceneCloud (scene_keypoints);
  gc_clusterer_.setModelSceneCorrespondences (model_scene_corrs);

  //gc_clusterer_.cluster_ (clustered_corrs);
  gc_clusterer_.recognize (std::get < 0 > (cluster_), std::get < 1 > (cluster_));
  return (cluster_);

}
