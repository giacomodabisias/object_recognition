#include "help.h"

void
ShowHelp (char *file_name)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Real time object recognition - Usage Guide                  *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage " << file_name << " model_filename_list [Options]" << std::endl << std::endl;
  std::cout << "Options" << std::endl;
  std::cout << "     -h                                 Show this help." << std::endl;
  std::cout << "     -ppfe                              Uses ppfe overriding all the other parameters." << std::endl;
  std::cout << "     -show_keypoints                    Show used keypoints." << std::endl;
  std::cout << "     -show_correspondences              Show used correspondences." << std::endl;
  std::cout << "     -filter                            Filter the cloud by color leaving only the points which are close to the model color." << std::endl;
  std::cout << "     -remove_outliers                   Remove ouliers from the scene." << std::endl;
  std::cout << "     -segment                           Segments the objects in the scene removing the major plane." << std::endl;
  std::cout << "     -log                               Saves the pose estimation error in a file called pose_error. " << std::endl;
  std::cout << "     -use_generalized_icp               Use the generalized ICP algorithm instead of the normal ICP algorithm. " << std::endl;  
  std::cout << "     --device val                       Specify a growing number starting from 0 if more devices are present" << std::endl;
  std::cout << "     --oni_file val                     Oni file path to use in offilne mode." << std::endl;
  std::cout << "     --filter_intensity val             Max distance between colors normalized between 0 and 1 (default 0.02)" << std::endl;
  std::cout << "     --descriptor_distance val          Descriptor max distance to be a match (default 0.25)" << std::endl;
  std::cout << "     --algorithm (hough|gc)             Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --keypoints (narf|sift|uniform|random|harris)    Keypoints detection algorithm (default uniform)." << std::endl;
  std::cout << "     --descriptors (shot|fpfh|pfh|pfhrgb|ppf)          Descriptor type (default shot)." << std::endl;
  std::cout << "     --model_ss val                     Model uniform sampling radius (default 0.005)" << std::endl;
  std::cout << "     --scene_ss val                     Scene uniform sampling radius (default 0.005)" << std::endl;
  std::cout << "     --rf_rad val:                      Hough reference frame radius (default 0.02)" << std::endl;
  std::cout << "     --descr_rad val                    Descriptor radius (default 0.03)" << std::endl;
  std::cout << "     --cg_size val                      Dimension of Hough's bins (default 0.007)" << std::endl;
  std::cout << "     --cg_thresh val                    Minimum number of positive votes for a match (default 6)" << std::endl;
  std::cout << "     --sift_min_scale val               (default 0.01)" << std::endl;
  std::cout << "     --sift_octaves val                 (default 3)" << std::endl;
  std::cout << "     --sift_scales_per_octave val       (default 2)" << std::endl;
  std::cout << "     --sift_min_contrast val            (default 0.3)" << std::endl;
  std::cout << "     --narf_support_size val            (default 0.02)" << std::endl;
  std::cout << "     --sac_seg_iter val                 Max iteration number of the ransac segmentation (default 1000)" << std::endl;
  std::cout << "     --reg_clustering_threshold val     Registration position clustering threshold (default 0.2)" << std::endl;
  std::cout << "     --reg_sampling_rate val            Ppfe registration sampling rate (default 10)" << std::endl;
  std::cout << "     --sac_seg_distance val             Ransac segmentation distance threshold (default 0.05)" << std::endl;
  std::cout << "     --max_inliers val                  Max number of inliers (default 40000)" << std::endl;
  std::cout << "     --random_scene_samples val         Number of random samples in the scene (default 1000) " << std::endl;
  std::cout << "     --random_model_samples val         Number of random samples in the model (default 1000) " << std::endl;
  std::cout << "     --harris_type val (HARRIS = 1|NOBLE = 2|LOWE = 3|TOMASI = 4|CURVATURE = 5)                 (default HARRIS) " << std::endl;
  std::cout << "     --descriptor_distance              Maximum distance between descriptors (default 0.25) " << std::endl;
  std::cout << "     --segmentation_threshold           Segmentation threshold for the plane recognition (default 0.01) " << std::endl;
  std::cout << "     --segmentation_iterations          Number of iteration of the segmenter (default 1000) " << std::endl;

}

void
ParseCommandLine (int argc, char *argv[])
{
  //Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    ShowHelp (argv[0]);
    exit (0);
  }

  //Program behavior
  if (pcl::console::find_switch (argc, argv, "-show_keypoints"))
    show_keypoints = true;
  if (pcl::console::find_switch (argc, argv, "-show_correspondences"))
    show_correspondences = true;
  if (pcl::console::find_switch (argc, argv, "-filter"))
    to_filter = true;
  if (pcl::console::find_switch (argc, argv, "-ppfe"))
    ppfe = true;
  if (pcl::console::find_switch (argc, argv, "-remove_outliers"))
    remove_outliers = true;
  if (pcl::console::find_switch (argc, argv, "-segment"))
    segment = true;
  if (pcl::console::find_switch (argc, argv, "-log"))
    error_log = true;
  if (pcl::console::find_switch (argc, argv, "-use_generalized_icp"))
    use_generalized_icp = true;

  std::string used_algorithm;
  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
  {
    if (used_algorithm.compare ("hough") == 0)
      use_hough = true;
    else if (used_algorithm.compare ("gc") == 0)
      use_hough = false;
    else
    {
      std::cout << "Wrong algorithm name.\n";
      ShowHelp (argv[0]);
      exit (-1);
    }
  }

  std::string used_keypoints;
  if (pcl::console::parse_argument (argc, argv, "--keypoints", used_keypoints) != -1)
  {
    if (used_keypoints.compare ("narf") == 0)
      narf = true;
    else if (used_keypoints.compare ("sift") == 0)
      sift = true;
    else if (used_keypoints.compare ("ransac") == 0)
      ransac = true;
    else if (used_keypoints.compare ("random") == 0)
      random_points = true;
    else if (used_keypoints.compare ("harris") == 0)
      harris = true;
    else if (used_keypoints.compare ("uniform") == 0)
      std::cout << "Using uniform sampling.\n";

  }

  std::string used_descriptors;
  if (pcl::console::parse_argument (argc, argv, "--descriptors", used_descriptors) != -1)
  {
    if (used_descriptors.compare ("shot") == 0)
      shot = true;
    else if (used_descriptors.compare ("fpfh") == 0)
      fpfh = true;
    else if (used_descriptors.compare ("ppf") == 0)
      ppf = true;
    else if (used_descriptors.compare ("ppfrgb") == 0)
      ppfrgb = true;
    else if (used_descriptors.compare ("pfh") == 0)
      pfh = true;
    else if (used_descriptors.compare ("pfhrgb") == 0)
      pfhrgb = true;
    else
    {
      std::cout << "Wrong descriptors type .\n";
      ShowHelp (argv[0]);
      exit (-1);
    }
  }

  //General parameters
  pcl::console::parse_argument (argc, argv, "--device", device_num);
  pcl::console::parse_argument (argc, argv, "--model_ss", model_ss);
  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss);
  pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad);
  pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad);
  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size);
  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh);
  pcl::console::parse_argument (argc, argv, "--sift_min_scale", min_scale);
  pcl::console::parse_argument (argc, argv, "--sift_octaves", n_octaves);
  pcl::console::parse_argument (argc, argv, "--sift_scales_per_octave", n_scales_per_octave);
  pcl::console::parse_argument (argc, argv, "--sift_min_contrast", min_contrast);
  pcl::console::parse_argument (argc, argv, "--narf_support_size", support_size);
  pcl::console::parse_argument (argc, argv, "--descriptor_distance", descriptor_distance);
  pcl::console::parse_argument (argc, argv, "--max_inliers", max_inliers);
  pcl::console::parse_argument (argc, argv, "--sac_seg_iter", sac_seg_iter);
  pcl::console::parse_argument (argc, argv, "--reg_clustering_threshold", reg_clustering_threshold);
  pcl::console::parse_argument (argc, argv, "--reg_sampling_rate", reg_sampling_rate);
  pcl::console::parse_argument (argc, argv, "--sac_seg_distance", sac_seg_distance);
  pcl::console::parse_argument (argc, argv, "--random_model_samples", random_model_samples);
  pcl::console::parse_argument (argc, argv, "--random_scene_samples", random_scene_samples);
  pcl::console::parse_argument (argc, argv, "--filter_intensity", filter_intensity);
  pcl::console::parse_argument (argc, argv, "--harris_type", harris_type);
  pcl::console::parse_argument (argc, argv, "--descriptor_distance", descriptor_distance);
  pcl::console::parse_argument (argc, argv, "--segmentation_threshold", segmentation_threshold);
  pcl::console::parse_argument (argc, argv, "--segmentation_iterations", segmentation_iterations);
  pcl::console::parse_argument (argc, argv, "--oni_file", oni_file);

}
