#include "Visualizer.h"
#include "define.h"
#include "Sampling.h"
#include "OpenniStreamer.h"
#include "Ppfe.h"
#include "KeyDes.hpp"
#include "Keypoints.hpp"
#include "Cluster.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include "Semaphore.h"
#include <thread>

void FindObject (pcl::PointCloud<PointType>::Ptr model, pcl::PointCloud<PointType>::Ptr scene, Semaphore& s, std::vector<ClusterType>& found_models) {
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr complete_scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());

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

  //Add the model to the filter so that the scene can be filtered using the model mean color
  if (to_filter)
  {
    filter = new ColorSampling();
    filter->AddCloud (*model);
  }

  //Remove outliers to clean the scene from sparse points
  if (remove_outliers)
  {
    sor = new pcl::StatisticalOutlierRemoval < PointType >();
    sor->setMeanK (50);
    sor->setStddevMulThresh (0.5);
  }

  std::cout << "calculating model normals... " << std::endl;
  //Calculate the model keypoints using the specified method
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

  //Start the main object recognition loop
  while (!s.ToStop()){ 

    init = std::clock();

    //Delete the main plane to reduce the number of points in the scene point cloud
    if (segment)
      scene = FindAndSubtractPlane (scene, segmentation_threshold, segmentation_iterations);
    //Filter the scene using the mean model color calculated before

    if (to_filter)
    {
      filter->FilterPointCloud (*scene, *scene);
    }
    //remove outliers from the scene to avoid sparse points
    if (remove_outliers)
    {
      sor->setInputCloud (scene);
      sor->filter (*scene);
    }

    //  Compute scene normals
    std::cout << "calculating scene normals... " << std::endl;
    scene_normals = norm.GetNormals (scene);

    //Calculate the scene keypoints using the specified method
    if (ppfe)
    {
      //PPFE
      cluster = ppfe_estimator->GetCluster (scene);
      scene_keypoints = ppfe_estimator->GetSceneKeypoints ();
      show_correspondences = false;
    }
    else
    {
      if (narf)
      {
        //NARF
        std::cout << "finding narf keypoints..." << std::endl;
        narf_estimator = new Narf();
        narf_estimator->GetKeypoints (scene, scene_keypoints);

      }
      else if (sift)
      {
        //SIFT
        std::cout << "finding sift keypoints..." << std::endl;
        sift_estimator = new Sift();
        sift_estimator->GetKeypoints (scene, scene_keypoints);

      }
      else if (ransac)
      {
        //RANSAC
        std::cout << "finding ransac keypoints..." << std::endl;
        ransac_estimator = new Ransac < pcl::SampleConsensusModelSphere < PointType >>();
        ransac_estimator->GetKeypoints (scene, scene_keypoints);

      }
      else if (harris)
      {
        //HARRIS
        std::cout << "finding harris keypoints..." << std::endl;
        harris_estimator = new Harris();
        harris_estimator->GetKeypoints (scene, scene_keypoints);

      }
      else if (random_points)
      {
        //RANDOM
        std::cout << "using random keypoints..." << std::endl;
        random->setInputCloud (scene);
        random->setSample (random_scene_samples);
        random->filter (*scene_keypoints);

      }
      else
      {
        //UNIFORM
        std::cout << "finding uniform sampled keypoints..." << std::endl;
        uniform.SetSamplingSize (scene_ss);
        uniform.GetKeypoints (scene, scene_keypoints);
      }

      std::cout << "\tfound " << scene_keypoints->points.size () << " keypoints in the scene and " << model_keypoints->points.size () << " in the model" << std::endl;

      //Calculate the correspondences between the model keypoints descriptors and the scene keypoints descriptors
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

      //Clustering the results and estimating an initial pose
      std::cout << "Starting to cluster..." << std::endl;
      if (use_hough)
      {
        //Hough3D
        cluster = hough->GetClusters ( scene, scene_keypoints, scene_normals, model_scene_corrs);
      }
      else
      {
        //GEOMETRIC CONSISTENCY
        cluster = gcg->GetClusters (model, scene, model_scene_corrs);
      }
    }
    std::cout << "\tFound " << std::get < 0 > (cluster).size () << " model instance/instances " << std::endl;
    if(std::get < 0 > (cluster).size () > 0)
      found_models.push_back(cluster);
    s.Notify2main();
    s.Wait4main();
  }
}



int
main (int argc, char** argv)
{
  //Parse input and set algorithm variables
  ParseCommandLine (argc, argv);
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr complete_scene (new pcl::PointCloud<PointType> ());
  std::vector<ClusterType> found_models;

  //Load the input model (n models but for now only one is used)
  std::vector < pcl::PointCloud < PointType > ::Ptr > model_list = ReadModels (argv);
  if (model_list.size () == 0)
  {
    std::cout << " no models loaded " << std::endl;
    return 1;
  }
  
  //Check if an oni file is specified as input or if the camera stream has to be used and initialize the correct stream.
  OpenniStreamer * openni_streamer;
  if(!oni_file.empty())
    openni_streamer = new OpenniStreamer(oni_file);
  else
    openni_streamer = new OpenniStreamer();

  //initialize all the fundamental datastructures
  int num_threads = model_list.size();
  Visualizer visualizer;
  Semaphore s(num_threads);
  std::vector<std::thread> thread_list(num_threads);

  //read first frame and launch the threads
  if(!openni_streamer->HasDataLeft())
      exit(0);
  scene = openni_streamer->GetCloud();
  copyPointCloud (*scene, *complete_scene);

  for(int i = 0; i < num_threads; ++i)
      thread_list[i] = std::thread(FindObject, model_list[i], scene, std::ref(s), std::ref(found_models));

  //start the main detection loop
  // 1- wait for the threads to find all the objects
  // 2- visualize the scene and the found models
  // 3- read a new scene
  // 4- wake up threads
  while(!visualizer.viewer_.wasStopped ()){
    //wait for the threads to complete
    s.Wait4threads();
    //Visualizing the model, scene and the estimated model position
    SetViewPoint (complete_scene);
    visualizer.Visualize (model_list, found_models, complete_scene);
    found_models.clear();
    //Grab a frame and create the pointcloud checking in case if the oni stream is finished
    if(!openni_streamer->HasDataLeft())
      break;
    scene = openni_streamer->GetCloud();
    //Copy the complete scene before filtering for visualization purpose
    copyPointCloud (*scene, *complete_scene);
    //wake up the threads
    s.Notify2threads();
  }

  //notifies all the threads top stop and waits for them to join
  s.SetStop();
  for(int i = 0; i < num_threads; ++i)
      thread_list[i].join();

  return (0);
}

