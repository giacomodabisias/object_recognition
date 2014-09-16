#include "find_function.h"

void FindObject (const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr&  original_scene, Semaphore& s, 
                 std::vector<ClusterType>& found_models,const int id, const float filter_intensity, const int icp_iteration,  ErrorWriter & e, const int & frame_index) {

  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());


  NormalEstimator norm;
  ClusterType cluster;
  Uniform uniform;
  Narf * narf_estimator;
  Sift * sift_estimator;
  Harris * harris_estimator;
  ColorSampling *filter;
  pcl::RandomSample < PointType > * random;
  pcl::StatisticalOutlierRemoval < PointType > *sor;
  Ransac < pcl::SampleConsensusModelSphere < PointType >> * ransac_estimator;
  Ppfe *ppfe_estimator;
  Hough * hough;
  GCG * gcg;
  ICP icp;
  struct timespec start, finish;
  double elapsed;

  // Add the model to the filter so that the scene can be filtered using the model mean color
  if (to_filter)
  {
    filter = new ColorSampling(filter_intensity);
    filter->AddCloud (*model);
  }

  // Remove outliers to clean the scene from sparse points
  if (remove_outliers)
  {
    sor = new pcl::StatisticalOutlierRemoval < PointType >();
    sor->setMeanK (50);
    sor->setStddevMulThresh (0.5);
  }

  std::cout << "calculating model normals... " << std::endl;
  // Calculate the model keypoints using the specified method
  if (ppfe)
  {       
    ppfe_estimator = new Ppfe(model);
    model_keypoints = ppfe_estimator->GetModelKeypoints ();    
  }else
  {
    std::cout << "model size " << model->points.size () << std::endl;
    model_normals = norm.GetNormals (model);
    if (random_points)
    {
      random = new pcl::RandomSample < PointType >();
      random->setInputCloud (model);
      random->setSeed (std::rand ());
      random->setSample (random_model_samples);
      random->filter (*model_keypoints);
    }
    else
    {
      uniform.SetSamplingSize (model_ss);
      uniform.GetKeypoints (model, model_keypoints);
    }
    if (use_hough)
      hough = new Hough(model, model_keypoints, model_normals);
    else
      gcg = new GCG();
  }

  // Start the main object recognition loop
  while (!s.ToStop()){
    clock_gettime(CLOCK_MONOTONIC, &start); 
    copyPointCloud (*original_scene, *scene);

    // Delete the main plane to reduce the number of points in the scene point cloud
    if (segment)
      scene = FindAndSubtractPlane (scene, segmentation_threshold, segmentation_iterations);
    // Filter the scene using the mean model color calculated before
    if (to_filter)
    {
      filter->FilterPointCloud (*scene, *scene);
    }
    // Remove outliers from the scene to avoid sparse points
    if (remove_outliers)
    {
      sor->setInputCloud (scene);
      sor->filter (*scene);
    }
    // Compute scene normals
    std::cout << "calculating scene normals... " << std::endl;
    scene_normals = norm.GetNormals (scene);
    // Calculate the scene keypoints using the specified method
    if (ppfe)
    {
      // PPFE
      cluster = ppfe_estimator->GetCluster (scene);
      scene_keypoints = ppfe_estimator->GetSceneKeypoints ();
      show_correspondences = false;
    }
    else
    {
      if (narf)
      {
        // NARF
        std::cout << "finding narf keypoints..." << std::endl;
        narf_estimator = new Narf();
        narf_estimator->GetKeypoints (scene, scene_keypoints);

      }
      else if (sift)
      {
        // SIFT
        std::cout << "finding sift keypoints..." << std::endl;
        sift_estimator = new Sift();
        sift_estimator->GetKeypoints (scene, scene_keypoints);

      }
      else if (ransac)
      {
        // RANSAC
        std::cout << "finding ransac keypoints..." << std::endl;
        ransac_estimator = new Ransac < pcl::SampleConsensusModelSphere < PointType >>();
        ransac_estimator->GetKeypoints (scene, scene_keypoints);
      }
      else if (harris)
      {
        // HARRIS
        std::cout << "finding harris keypoints..." << std::endl;
        harris_estimator = new Harris();
        harris_estimator->GetKeypoints (scene, scene_keypoints);
      }
      else if (random_points)
      {
        // RANDOM
        std::cout << "using random keypoints..." << std::endl;
        random->setInputCloud (scene);
        random->setSample (random_scene_samples);
        random->filter (*scene_keypoints);
      }
      else
      {
        // UNIFORM
        std::cout << "finding uniform sampled keypoints..." << std::endl;
        uniform.SetSamplingSize (scene_ss);
        uniform.GetKeypoints (scene, scene_keypoints);
      }

      std::cout << "\tfound " << scene_keypoints->points.size () << " keypoints in the scene and " << model_keypoints->points.size () << " in the model" << std::endl;

      // Calculate the correspondences between the model keypoints descriptors and the scene keypoints descriptors
      pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
      if (fpfh)
      {
        std::cout << "using fpfh descriptors" << std::endl;
        KeyDes<pcl::FPFHSignature33, pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> > est (model, model_keypoints, scene, scene_keypoints, model_normals, scene_normals);
        model_scene_corrs = est.Run ();
      }
      else if (pfh)
      {
        std::cout << "using pfh descriptors" << std::endl;
        KeyDes<pcl::PFHSignature125, pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> > est (model, model_keypoints, scene, scene_keypoints, model_normals, scene_normals);
        model_scene_corrs = est.Run ();
      }
      else if (pfhrgb)
      {
        std::cout << "using pfhrgb descriptors" << std::endl;
        KeyDes<pcl::PFHRGBSignature250, pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> > est (model, model_keypoints, scene, scene_keypoints, model_normals, scene_normals);
        model_scene_corrs = est.Run ();
      }
      else if (ppf)
      {
        std::cout << "using ppf descriptors" << std::endl;
        KeyDes<pcl::PPFSignature, pcl::PPFEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PPFSignature> > est (model, model_keypoints, scene, scene_keypoints, model_normals, scene_normals);
        model_scene_corrs = est.Run ();
      }
      else if (ppfrgb)
      {
        std::cout << "using ppfrgb descriptors" << std::endl;
        KeyDes<pcl::PPFRGBSignature, pcl::PPFRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PPFRGBSignature> > est (model, model_keypoints, scene, scene_keypoints, model_normals, scene_normals);
        model_scene_corrs = est.Run ();
      }
      else if (shot)
      {
        std::cout << "using shot descriptors" << std::endl;
        KeyDes<pcl::SHOT352, pcl::SHOTEstimationOMP<PointType, NormalType, pcl::SHOT352> > est (model, model_keypoints, scene, scene_keypoints, model_normals, scene_normals);
        model_scene_corrs = est.Run ();
      }
      else if (_3dsc)
      {
        std::cout << "using 3dsc descriptors" << std::endl;
        ShapeContext3D est (model_keypoints, scene_keypoints, model_normals, scene_normals);
        model_scene_corrs = est.run ();
      }
      else if (usc)
      {
        std::cout << "using usc descriptors" << std::endl;
        Usc est (model_keypoints, scene_keypoints);
        model_scene_corrs = est.run ();
      }/*
      else if (spin)
      {
        std::cout << "using spin descriptors" << std::endl;
        Spin est (model_keypoints, scene_keypoints, model_normals, scene_normals);           
        model_scene_corrs = est.run ();
      }
       else if (rift)
      {
        std::cout << "using rift descriptors" << std::endl;
        Rift est (model_keypoints, scene_keypoints, model_normals, scene_normals);           
        model_scene_corrs = est.run ();
      }*/
  

      // Clustering the results and estimating an initial pose
      std::cout << "Starting to cluster..." << std::endl;
      if (use_hough)
      {
        // Hough3D
        cluster = hough->GetClusters ( scene, scene_keypoints, scene_normals, model_scene_corrs);
      }
      else
      {
        // Geometric Consistency
        cluster = gcg->GetClusters (model, scene, model_scene_corrs);
      }
    }
    std::cout << "\tFound " << std::get < 0 > (cluster).size () << " model instance/instances " << std::endl;
    if(std::get < 0 > (cluster).size () > 0){
        clock_gettime(CLOCK_MONOTONIC, &finish);
        elapsed = (finish.tv_sec - start.tv_sec);
        elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
        if(use_icp){
            pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
            std::cout << "\t USING ICP"<<std::endl;
            pcl::transformPointCloud (*model, *rotated_model, (std::get < 0 > (cluster)[0]));
       
            Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();
            for(int i = 0; i< icp_iteration; ++i){
            //while(!icp.HasConverged()){
              icp.Align (rotated_model, original_scene);
              tmp = icp.transformation_ * tmp;  
            }
            std::get < 0 > (cluster)[0] =  tmp * std::get < 0 > (cluster)[0];
            
            if(error_log)
                e.WriteError(icp.transformation_, icp.fitness_score_, id, elapsed, frame_index);
          }
      found_models[id] = cluster; 
    }
    s.Notify2main();
    s.Wait4main();
  }
}