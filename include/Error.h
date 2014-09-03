#ifndef ERROR_H
#define ERROR_H
#include "utils.h"
#include <mutex>





class ErrorWriter
{
public:
  std::ofstream es_;
  std::mutex mutex_;
  Eigen::Matrix3f rotation_;
  Eigen::Matrix3f tmp_;
  Eigen::Vector3f traslation_;
  float rotation_error_;
  float traslation_error_;
  error e_;

  ErrorWriter();
  ~ErrorWriter();

  void 
  WriteError(const Eigen::Matrix4f transformation, float fitness, int id, double end);

  void 
  WriteError( float fitness, double end);

  void
  GetRototraslationError (const Eigen::Matrix4f transformation);

};

#endif