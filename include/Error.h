#ifndef ERROR_H
#define ERROR_H
#include "utils.h"


error
GetRototraslationError (const Eigen::Matrix4f transformation);

class ErrorWriter
{
public:
  std::ofstream es_;

  ErrorWriter();

  void 
  WriteError(error e, float fitness);

  void 
  WriteError(float fitness);

};

#endif