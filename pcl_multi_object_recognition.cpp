#include "find_function.h"
#include "OpenniStreamer.h"

int
main (int argc, char** argv)
{
  //Parse input and set algorithm variables
  ParseCommandLine (argc, argv);
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr complete_scene (new pcl::PointCloud<PointType> ());


  //Load the input model (n models but for now only one is used)
  std::vector < pcl::PointCloud < PointType > ::Ptr > model_list = ReadModels (argv);
  if (model_list.size () == 0)
  {
    std::cout << " no models loaded " << std::endl;
    return 1;
  }
  
  //Check if an oni file is specified as input or if the camera stream has to be used and initialize the correct stream.
  OpenniStreamer * openni_streamer;
  if(!oni_file.empty())
    openni_streamer = new OpenniStreamer(oni_file);
  else
    openni_streamer = new OpenniStreamer();

  //initialize all the fundamental datastructures
  int num_threads = model_list.size();
  Visualizer visualizer;
  Semaphore s(num_threads);
  std::vector<std::thread> thread_list(num_threads);
  std::vector<ClusterType> found_models(num_threads);


  //read first frame and launch the threads
  if(!openni_streamer->HasDataLeft())
      exit(0);
  scene = openni_streamer->GetCloud();
  copyPointCloud (*scene, *complete_scene);

  for(int i = 0; i < num_threads; ++i)
      thread_list[i] = std::thread(FindObject, model_list[i], std::ref(scene), std::ref(s), std::ref(found_models), i);
    

  //start the main detection loop
  // 1- wait for the threads to find all the objects
  // 2- visualize the scene and the found models
  // 3- read a new scene
  // 4- wake up threads
  while(!visualizer.viewer_.wasStopped ()){
    //wait for the threads to complete
    s.Wait4threads();
    //Visualizing the model, scene and the estimated model position
    SetViewPoint (complete_scene);
    visualizer.Visualize (model_list, found_models, complete_scene);
    
    //Grab a frame and create the pointcloud checking in case if the oni stream is finished
    if(!openni_streamer->HasDataLeft())
      break;
    scene = openni_streamer->GetCloud();
    copyPointCloud (*scene, *complete_scene);
    //wake up the threads
    s.Notify2threads();
  }

  //notifies all the threads top stop and waits for them to join
  s.SetStop();
  for(int i = 0; i < num_threads; ++i)
      thread_list[i].join();

  return (0);
}

