#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "Error.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/transforms.h>


class Visualizer
{
  public:
    pcl::visualization::PCLVisualizer viewer_;
    int iter_;
    bool clean_;
    int r, g;
    std::stringstream ss_cloud_;
    std::vector<std::string> to_remove_;

    Visualizer ();

    void
    Visualize ( std::vector<pcl::PointCloud<PointType>::Ptr> model_list,  std::vector<ClusterType> found_models,  pcl::PointCloud<PointType>::Ptr scene);
};

#endif