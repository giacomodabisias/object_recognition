#ifndef DEFINE_H
#define DEFINE_H

#include <pcl/features/board.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


//defines
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType>::ConstPtr CloudConstPtr;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef std::tuple<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >, std::vector<pcl::Correspondences>> ClusterType;
typedef std::tuple<float, float> error;


extern std::clock_t init;
extern std::string oni_file;
extern const Eigen::Vector4f SUBSAMPLING_LEAF_SIZE;

//Algorithm params
extern float model_ss;
extern float scene_ss;
extern float rf_rad;
extern float descr_rad;
extern float cg_size;
extern float cg_thresh;
extern float sac_seg_iter;
extern float reg_sampling_rate;
extern float sac_seg_distance;
extern float reg_clustering_threshold;
extern float max_inliers;
extern float min_scale;
extern float min_contrast;
extern float support_size;
extern float filter_intensity;
extern float descriptor_distance;
extern float segmentation_threshold;
extern float normal_estimation_search_radius;
extern int segmentation_iterations;
extern int n_octaves;
extern int n_scales_per_octave;
extern int random_scene_samples;
extern int random_model_samples;
extern int distance;
extern int harris_type;
extern int device_num;
extern bool narf;
extern bool random_points;
extern bool sift;
extern bool harris;
extern bool fpfh;
extern bool pfh;
extern bool pfhrgb;
extern bool ppf;
extern bool ppfrgb;
extern bool shot;
extern bool ransac;
extern bool ppfe;
extern bool first;
extern bool show_keypoints;
extern bool show_correspondences;
extern bool use_hough;
extern bool to_filter;
extern bool show_filtered;
extern bool remove_outliers;
extern bool use_icp;
extern bool segment;
extern bool error_log;
extern bool use_generalized_icp;


#endif