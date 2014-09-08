#ifndef HELP_H
#define HELP_H

#include <iostream>
#include "define.h"
#include <pcl/console/parse.h>


//Shows help when argument list is invalid
inline void
ShowHelp (char *file_name);

//Sets all the algorithms variables using the input arguments
void
ParseCommandLine (int argc, char *argv[]);

//Displayes help during visualization
inline void
ShowKeyHelp ()
{
  std::cout << "Press q to increase the Hough thresh by 1" << std::endl;
  std::cout << "Press w to decrease the Hough thresh by 1" << std::endl;
  std::cout << "Press a to increase Hough bin size by 0.001" << std::endl;
  std::cout << "Press s to decrease Hough bin size by 0.001" << std::endl;
  std::cout << "Press z to increase the scene sampling size" << std::endl;
  std::cout << "Press x to decrease the scene sampling size" << std::endl;
  std::cout << "Press p to print the actual parameters" << std::endl;
  std::cout << "Press k to toggle filtered mode" << std::endl;
  std::cout << "Press i to toggle icp alignment" << std::endl;
  std::cout << "Press n to incraese aquired distance" << std::endl;
  std::cout << "Press m to incraese aquired distance" << std::endl;
  std::cout << "Press d to lower segmentation threshold " << std::endl;
  std::cout << "Press f to increase segmentation threshold" << std::endl;
  std::cout << "Press l to lower filtering" << std::endl;
  std::cout << "Press k to increase filtering" << std::endl;
}

#endif