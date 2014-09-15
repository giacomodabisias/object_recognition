#include "Ppfe.h"


Ppfe::Ppfe (pcl::PointCloud<PointType>::Ptr model)
{

  hashmap_search_ = boost::make_shared <pcl::PPFHashMapSearch> (12.0f / 180.0f * float (M_PI), 0.05f);
  cloud_model_ppf_ = boost::make_shared<pcl::PointCloud<pcl::PPFSignature>> ();
  inliers_ = boost::make_shared<pcl::PointIndices> ();
  model_xyz_ = boost::make_shared<pcl::PointCloud<XYZType>> ();
  copyPointCloud (*model, *model_xyz_);
  coefficients_ = boost::make_shared<pcl::ModelCoefficients> ();
  cloud_scene_ = boost::make_shared<pcl::PointCloud<XYZType>> ();
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setMaxIterations (sac_seg_iter);
  extract_.setNegative (true);
  ppf_registration_.setSceneReferencePointSamplingRate (reg_sampling_rate);  //10
  ppf_registration_.setPositionClusteringThreshold (reg_clustering_threshold);  //0.2f
  ppf_registration_.setRotationClusteringThreshold (30.0f / 180.0f * float (M_PI));
  cloud_model_input_ = SubsampleAndCalculateNormals (model_xyz_);
  pcl::PPFEstimation < pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature > ppf_estimator;
  ppf_estimator.setInputCloud (cloud_model_input_);
  ppf_estimator.setInputNormals (cloud_model_input_);
  ppf_estimator.compute (*cloud_model_ppf_);
  hashmap_search_->setInputFeatureCloud (cloud_model_ppf_);
  ppf_registration_.setSearchMethod (hashmap_search_);
  ppf_registration_.setInputSource (cloud_model_input_);

}

ClusterType
Ppfe::GetCluster (pcl::PointCloud<PointType>::Ptr scene)
{
  seg_.setDistanceThreshold (sac_seg_distance);
  copyPointCloud (*scene, *cloud_scene_);
  nr_points_ = unsigned (cloud_scene_->points.size ());
  while (cloud_scene_->points.size () > 0.3 * nr_points_) 
  {
    seg_.setInputCloud (cloud_scene_);
    seg_.segment (*inliers_, *coefficients_);
    //PCL_INFO ("Plane inliers: %u\n", inliers_->indices.size ());
    if (inliers_->indices.size () < max_inliers)
      break;
    extract_.setInputCloud (cloud_scene_);
    extract_.setIndices (inliers_);
    extract_.filter (*cloud_scene_);
  }

  cloud_scene_input_ = SubsampleAndCalculateNormals (cloud_scene_);


  ppf_registration_.setInputTarget (cloud_scene_input_);
  ppf_registration_.align (cloud_output_subsampled_);
  // pcl::PointCloud<XYZType>::Ptr cloud_output_subsampled_xyz (new pcl::PointCloud<XYZType> ());
  //for (size_t i = 0; i < cloud_output_subsampled_.points.size (); ++i)
  //  cloud_output_subsampled_xyz->points.push_back ( XYZType (cloud_output_subsampled_.points[i].x, cloud_output_subsampled_.points[i].y, cloud_output_subsampled_.points[i].z));
  mat_ = ppf_registration_.getFinalTransformation ();
  std::vector <pcl::Correspondences> cor_tmp;

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 4, 0, 4, 4>>>mat_tmp;
  mat_tmp.push_back (mat_);
  ClusterType cluster_ = std::make_tuple (mat_tmp, cor_tmp);
  inliers_->indices.clear ();

  return (cluster_);
}

pcl::PointCloud<PointType>::Ptr
Ppfe::GetModelKeypoints ()
{
  pcl::PointCloud<PointType>::Ptr tmp (new pcl::PointCloud<PointType> ());
  copyPointCloud (*cloud_model_input_, *tmp);
  return (tmp);
}

pcl::PointCloud<PointType>::Ptr
Ppfe::GetSceneKeypoints ()
{
  pcl::PointCloud<PointType>::Ptr tmp (new pcl::PointCloud<PointType> ());
  copyPointCloud (*cloud_scene_, *tmp);
  return (tmp);
}

pcl::PointCloud<pcl::PointNormal>::Ptr
SubsampleAndCalculateNormals (pcl::PointCloud<XYZType>::Ptr cloud)
{
  pcl::PointCloud<XYZType>::Ptr cloud_subsampled (new pcl::PointCloud<XYZType> ());
  pcl::VoxelGrid <XYZType> subsampling_filter;
  subsampling_filter.setInputCloud (cloud);
  subsampling_filter.setLeafSize (SUBSAMPLING_LEAF_SIZE);
  subsampling_filter.filter (*cloud_subsampled);

  pcl::PointCloud<NormalType>::Ptr cloud_subsampled_normals (new pcl::PointCloud<NormalType> ());
  pcl::NormalEstimationOMP <XYZType, NormalType > normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_subsampled);
  pcl::search::KdTree<XYZType>::Ptr search_tree (new pcl::search::KdTree<XYZType>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch (normal_estimation_search_radius);
  normal_estimation_filter.compute (*cloud_subsampled_normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_subsampled_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
  concatenateFields (*cloud_subsampled, *cloud_subsampled_normals, *cloud_subsampled_with_normals);

  PCL_INFO ("Cloud dimensions before / after subsampling: %u / %u\n", cloud->points.size (), cloud_subsampled->points.size ());
  return (cloud_subsampled_with_normals);
}