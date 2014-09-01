#include "Error.h"

error
GetRototraslationError (const Eigen::Matrix4f transformation)
{
  Eigen::Matrix3f rotation;
  Eigen::Vector3f traslation;
  float rotation_error;
  float traslation_error;
  error e;

  rotation << transformation(0,0), transformation(0,1), transformation(0,2),
              transformation(1,0), transformation(1,1), transformation(1,2),
              transformation(2,0), transformation(2,1), transformation(2,2);  
  traslation << transformation(0,3), transformation(1,3), transformation(2,3);

  traslation_error = traslation.norm();
  std::get < 1 > (e) = traslation_error;
  std::cout << "rotation" <<std::endl;
  std::cout << rotation << std::endl;

  Eigen::Matrix3f tmp =  rotation.transpose();
  float theta = acos((tmp(0,0) + tmp(1,1) + tmp(2,2) - 1) / 2);
  rotation_error = frobeniusNorm((theta / (2 * sin(theta) )) * ( tmp - tmp.transpose()) );

  return (e);
}

ErrorWriter::ErrorWriter() 
{
  es_.open("pose_error.txt");
}

void 
ErrorWriter::WriteError(error e, float fitness)
{ if( std::isnan(std::get < 0 > (e)))
    WriteError(fitness);
  else
    es_ << std::get < 0 > (e) << ", " << std::get < 1 > (e) << ", " << double(std::clock() - init) / CLOCKS_PER_SEC << ", " << fitness << std::endl;
}

void 
ErrorWriter::WriteError(float fitness)
{
  es_ << "onf, onf "  << double(std::clock() - init) / CLOCKS_PER_SEC << ", " << fitness << std::endl;
}

