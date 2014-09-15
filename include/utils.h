#ifndef UTILS_H
#define UTILS_H
#include "help.h"
#include <pcl/visualization/keyboard_event.h>
//#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <sstream>

//Returns the frobenious norm of the input matrix
double 
frobeniusNorm(const Eigen::Matrix3f matrix);

//Check if a key was pressend during visualization and applies effects
void
KeyboardEventOccurred (const pcl::visualization::KeyboardEvent &event);

//Sets the correct cloud view point
inline void
SetViewPoint (pcl::PointCloud<PointType>::Ptr cloud)
{

  cloud->sensor_origin_.setZero ();
  cloud->sensor_orientation_.w () = 0.0;
  cloud->sensor_orientation_.x () = 1.0;
  cloud->sensor_orientation_.y () = 0.0;
  cloud->sensor_orientation_.z () = 0.0;
}

//Load the input models and returns a list of point clouds
std::vector<pcl::PointCloud<PointType>::Ptr>
ReadModels (char** argv, std::vector<float>& filters, std::vector<int>& icp_iterations);

//Prints the transformation matrix and traslation vector
void
PrintTransformation (const ClusterType cluster);

class NormalEstimator
{
  public:

    //pcl::NormalEstimationOMP<PointType, NormalType> norm_est_;
    pcl::NormalEstimation<PointType, NormalType> norm_est_;

    NormalEstimator ();

    NormalEstimator (int n_neighbours);

    pcl::PointCloud<NormalType>::Ptr
    GetNormals (const pcl::PointCloud<PointType>::Ptr cloud);
};

#endif