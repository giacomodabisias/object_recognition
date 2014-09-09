#include "Error.h"

void
ErrorWriter::GetRototraslationError (const Eigen::Matrix4f transformation)
{
  rotation_ << transformation(0,0), transformation(0,1), transformation(0,2),
              transformation(1,0), transformation(1,1), transformation(1,2),
              transformation(2,0), transformation(2,1), transformation(2,2);  
  traslation_ << transformation(0,3), transformation(1,3), transformation(2,3);

  traslation_error_ = traslation_.norm();
  std::get < 0 > (e_) = traslation_error_;
  tmp_ =  rotation_.transpose();
  float theta = acos((tmp_(0,0) + tmp_(1,1) + tmp_(2,2) - 1) / 2);
  rotation_error_ = frobeniusNorm((theta / (2 * sin(theta) )) * ( tmp_ - tmp_.transpose()) );
  std::get < 1 > (e_) = rotation_error_;

}

ErrorWriter::ErrorWriter() 
{
  es_.open("pose_error.txt");
  es_ << "frame_index, thread_id, thread id, translation error, rotation error, recognition time, icp fitness score" <<std::endl;
}

void 
ErrorWriter::WriteError(const Eigen::Matrix4f transformation, float fitness, int id, double end, int frame_index)
{ 
  GetRototraslationError(transformation);
  std::unique_lock<std::mutex> lock(mutex_);
  es_ << frame_index << ", " << id << ", " << std::get < 0 > (e_) << ", " << std::get < 1 > (e_) << ", " << end << ", " << fitness << std::endl;
}

void 
ErrorWriter::WriteError( float fitness, double end)
{
  std::unique_lock<std::mutex> lock(mutex_);
  es_ << "onf, onf "  << end << ", " << fitness << std::endl;
}

ErrorWriter::~ErrorWriter()
{
  es_.close();
}

