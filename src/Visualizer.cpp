#include "Visualizer.h"



Visualizer::Visualizer () : iter_ (0), clean_ (true), r(255), g(0)
{
  viewer_.registerKeyboardCallback (KeyboardEventOccurred);
}

void
Visualizer::Visualize ( const std::vector<pcl::PointCloud<PointType>::Ptr> model_list, const std::vector<ClusterType> found_models, const pcl::PointCloud<PointType>::Ptr scene)
{

  if (!clean_)
  {
    for (auto s : to_remove_)
      viewer_.removeShape (s);
    clean_ = true;
    to_remove_.clear ();
  }

  if (iter_ == 0)
  {
    viewer_.addPointCloud (scene, "scene_cloud");
  }
  else
    viewer_.updatePointCloud (scene, "scene_cloud");

  for (size_t i = 0; i < found_models.size (); ++i)
  {
    clean_ = false;
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*(model_list[i]), *rotated_model, (std::get < 0 > (found_models[i]))[0]);
    /*if(use_icp){
    ICPRegistration icp ;
    icp.Align (rotated_model, scene);
    }*/
    SetViewPoint (rotated_model);

    ss_cloud_ << "instance" << i;
    to_remove_.push_back (ss_cloud_.str ());

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, r, g, 0);
    
    viewer_.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud_.str ());
  }

  viewer_.spinOnce ();
  iter_++;
}
