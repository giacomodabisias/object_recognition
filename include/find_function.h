#ifndef OBJECT_RECOGNITION_FIND_FUNCTION_H
#define OBJECT_RECOGNITION_FIND_FUNCTION_H

#include <vector>
#include <pcl/features/board.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Error.h"
#include "Semaphore.h"

typedef pcl::PointXYZRGB PointType;
typedef std::tuple<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >, std::vector<pcl::Correspondences>> ClusterType;

void FindObject (const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr&  original_scene, Semaphore& s, 
	             std::vector<ClusterType>& found_models,const  int id, const float filter, const int icp_iteration, ErrorWriter & e, const int &frame_index);

#endif