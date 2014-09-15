#include "Sampling.h"

pcl::PointCloud<PointType>::Ptr
FindAndSubtractPlane (const pcl::PointCloud<PointType>::Ptr input, float distance_threshold, float max_iterations)
{
  // Find the dominant plane
  pcl::SACSegmentation<PointType> seg;
  seg.setOptimizeCoefficients (false);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distance_threshold);
  seg.setMaxIterations (max_iterations);
  seg.setInputCloud (input);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  seg.segment (*inliers, *coefficients);

  // Extract the inliers
  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud (input);
  extract.setIndices (inliers);
  extract.setNegative (true);
  pcl::PointCloud<PointType>::Ptr output (new pcl::PointCloud<PointType> ());
  extract.filter (*output);

  return (output);
}

ColorSampling::ColorSampling (float filter): filter_(filter)
{
  Clear ();
  rgb2yuv << 0.229, 0.587, 0.114, -0.14713, -0.28886, 0.436, 0.615, -0.51499, -0.10001;
}

void
ColorSampling::Clear ()
{
  avg_u_ = 0, avg_v_ = 0;
}

void
ColorSampling::AddCloud (const pcl::PointCloud<PointType> &cloud)
{
  Clear ();
  float u, v;
  for (auto point : cloud.points)
  {
    RGBtoYUV (point, u, v);
    avg_u_ += u;
    avg_v_ += v;
  }
  avg_u_ = avg_u_ / cloud.points.size ();
  avg_v_ = avg_v_ / cloud.points.size ();
}

void
ColorSampling::ColorSampling::FilterPointCloud (const pcl::PointCloud<PointType> &in_cloud, pcl::PointCloud<PointType> &out_cloud)
{
  pcl::PointCloud < pcl::PointXYZRGB > cloud;
  int points = in_cloud.points.size ();

  for (auto point : in_cloud.points)
  {
    if (!ToFilter (point))
      cloud.points.push_back (point);
  }
  out_cloud = cloud;
  std::cout << "Point number: \n\t Original point cloud: " << points << " \n\t Filtered point cloud: " << cloud.points.size () << std::endl;
}

void
ColorSampling::PrintColorInfo ()
{
  std::cout << "avg U: " << avg_u_ << " V: " << avg_v_ << std::endl;
}

Eigen::Matrix3f rgb2yuv;

float avg_u_, avg_v_;

bool
ColorSampling::ToFilter (const PointType &point)
{
  float u, v;
  RGBtoYUV (point, u, v);
  float distance = sqrt (pow (u - avg_u_, 2) + pow (v - avg_v_, 2));
  if (distance < filter_intensity)
    return (false);
  else
    return (true);
}

void
ColorSampling::RGBtoYUV (const PointType &point, float &u, float &v)
{
  Eigen::Vector3f rgb ((float) point.r / 255, (float) point.g / 255, (float) point.b / 255);
  Eigen::Vector3f yuv = rgb2yuv * rgb;
  u = yuv.y ();
  v = yuv.z ();
}

DownSampler::DownSampler ()
{
  down_sampler_.setLeafSize (0.001, 0.001, 0.001);
}

DownSampler::DownSampler (float x, float y, float z)
{
  down_sampler_.setLeafSize (x, y, z);
}

void
DownSampler::SetSampleSize (float x, float y, float z)
{
  down_sampler_.setLeafSize (x, y, z);
}

void
DownSampler::DownSample (pcl::PointCloud<PointType>::Ptr cloud)
{
  down_sampler_.setInputCloud (cloud);
  down_sampler_.filter (*cloud);
}
