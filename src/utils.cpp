#include "utils.h"

double frobeniusNorm(const Eigen::Matrix3f matrix)
{
    double result = 0.0;
    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
        {
            double value = matrix(i, j);
            result += value * value;
        }
    }
    return sqrt(result);
}

void
KeyboardEventOccurred (const pcl::visualization::KeyboardEvent &event)
{
  std::string pressed;
  pressed = event.getKeySym ();
  if (event.keyDown ())
  {
    if (pressed == "q")
    {
      cg_thresh++;
      std::cout << "\tcg_thresh increased to " << cg_thresh << std::endl;
    }
    else if (pressed == "w")
    {
      cg_thresh--;
      std::cout << "\tcg_thresh decreased to " << cg_thresh << std::endl;
    }
    else if (pressed == "a")
    {
      cg_size += 0.001;
      std::cout << "\tcg_size increased to " << cg_size << std::endl;
    }
    else if (pressed == "s")
    {
      cg_size -= 0.001;
      std::cout << "\tcg_size decreased to " << cg_size << std::endl;
    }
    else if (pressed == "z")
    {
      scene_ss += 0.001;
      std::cout << "\tscene sampling size increased to " << scene_ss << std::endl;
    }
    else if (pressed == "x")
    {
      scene_ss -= 0.001;
      std::cout << "\tscene sampling size decreased to " << scene_ss << std::endl;
    }
    else if (pressed == "e")
    {
      sac_seg_distance += 0.001;
      std::cout << "\t sac segmentation distance increased to " << sac_seg_distance << std::endl;
    }
    else if (pressed == "e")
    {
      sac_seg_distance -= 0.001;
      std::cout << "\t sac segmentation distance decreased to " << sac_seg_distance << std::endl;
    }
    else if (pressed == "v")
    {
      show_filtered = !show_filtered;
    }
    else if (pressed == "n")
    {
      distance += 100;
    }
    else if (pressed == "d")
    {
      segmentation_threshold += 0.01;
    }
    else if (pressed == "f")
    {
      if (segmentation_threshold > 0)
        segmentation_threshold -= 0.01;
    }
    else if (pressed == "l")
    {
      filter_intensity += 0.01;
    }
    else if (pressed == "k")
    {
      if (filter_intensity > 0)
        filter_intensity -= 0.01;
    }
    else if (pressed == "m")
    {
      if (distance > 200)
        distance -= 100;
    }
    else if (pressed == "i")
    {
      use_icp = !use_icp;
    }
    else if (pressed == "h")
    {
      ShowKeyHelp ();
    }
  }
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
ReadModels (char** argv)
{
  pcl::PCDReader reader;

  std::vector < pcl::PointCloud < pcl::PointXYZRGB > ::Ptr > cloud_models;

  std::ifstream pcd_file_list (argv[1]);
  if(pcd_file_list.is_open()){
    while (!pcd_file_list.eof ())
    {
      char str[512];
      pcd_file_list.getline (str, 512);
      if (std::strlen (str) > 2)
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
        reader.read (str, *cloud);
        ///SetViewPoint(cloud);
        cloud_models.push_back (cloud);
        PCL_INFO ("Model read: %s\n", str);
      }
    }
    std::cout << "all loaded" << std::endl;
  }
  else{
    std::cout << "model file list not existant" <<std::endl;
    exit(0);
  }
  return (std::move (cloud_models));
}

void
PrintTransformation (const ClusterType cluster)
{
  for (size_t i = 0; i < std::get < 0 > (cluster).size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << std::get < 1 > (cluster)[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = std::get < 0 > (cluster)[i].block<3, 3> (0, 0);
    Eigen::Vector3f translation = std::get < 0 > (cluster)[i].block<3, 1> (0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0, 0), rotation (0, 1), rotation (0, 2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1, 0), rotation (1, 1), rotation (1, 2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2, 0), rotation (2, 1), rotation (2, 2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }
}

NormalEstimator::NormalEstimator() {
  norm_est_.setKSearch (10);
}

NormalEstimator::NormalEstimator(int n_neighbours): NormalEstimator::NormalEstimator() { 
  norm_est_.setKSearch (n_neighbours);
}

pcl::PointCloud<NormalType>::Ptr NormalEstimator::GetNormals(const pcl::PointCloud<PointType>::Ptr cloud){

  pcl::PointCloud<NormalType>::Ptr normals(new pcl::PointCloud<NormalType> ());
  norm_est_.setInputCloud (cloud);
  norm_est_.compute (*normals);

  return normals;
}
