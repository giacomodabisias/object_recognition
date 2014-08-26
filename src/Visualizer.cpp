#include "Visualizer.h"



Visualizer::Visualizer () : iter_ (0), clean_ (true), r(255), g(0)
{
  //viewer_.registerKeyboardCallback (KeyboardEventOccurred);
  if(use_generalized_icp)
    icp_ = new GeneralizedICPRegistration();
  else
    icp_ = new ICPRegistration();

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
    if (use_icp && scene->points.size() > 20)
    {
      icp_->Align (rotated_model, scene);
      //pcl::transformPointCloud (*rotated_model, *rotated_model, transformation);
    }
    SetViewPoint (rotated_model);

    ss_cloud_ << "instance" << i;
    to_remove_.push_back (ss_cloud_.str ());

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, r, g, 0);

    viewer_.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud_.str ());
  }
  /*
  if(error_log && use_icp)
  {
    if(icp_->fitness_score_ < 0.00095 && filtered_scene->points.size() > 20)
    {
      g = 255;
      r = 0;
      e.WriteError(GetRototraslationError((icp_->transformation_)), icp_->fitness_score_);
    }
    else 
    {
      r  = 255;
      g = 0;
      e.WriteError(icp_->fitness_score_);
    }
  }
  */
  viewer_.spinOnce ();
  iter_++;
}
