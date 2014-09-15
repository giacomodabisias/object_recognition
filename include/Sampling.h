#ifndef SAMPLING_H
#define SAMPLINH_H

#include "define.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>


//Removes the principal plane from a point cloud and returns the new point cloud
pcl::PointCloud<PointType>::Ptr
FindAndSubtractPlane (const pcl::PointCloud<PointType>::Ptr input, float distance_threshold, float max_iterations);

class ColorSampling
{
  public:
    Eigen::Matrix3f rgb2yuv;
    float avg_u_, avg_v_;
    float filter_;

    ColorSampling (float filter);

    void
    Clear ();

    void
    AddCloud (const pcl::PointCloud<PointType> &cloud);

    void
    FilterPointCloud (const pcl::PointCloud<PointType> &in_cloud, pcl::PointCloud<PointType> &out_cloud);

    void
    PrintColorInfo ();

    bool
    ToFilter (const PointType &point);

    void
    RGBtoYUV (const PointType &point, float &u, float &v);
};

class DownSampler
{
  public:
    pcl::VoxelGrid<PointType> down_sampler_;

    DownSampler ();
    DownSampler (float x, float y, float z);

    void
    SetSampleSize (float x, float y, float z);

    void
    DownSample (pcl::PointCloud<PointType>::Ptr cloud);
};

#endif