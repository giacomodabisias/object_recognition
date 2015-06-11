#ifndef OPENNI_STREAMER_H
#define OPENNI_STREAMER_H

#include <pcl/filters/passthrough.h>
#include "openni2_grabber.hpp"
#include "pcl/io/openni2_grabber.h"
#include "define.h"
#include "OpenNI.h"


class OpenniStreamer
{
  public:
    openni::Device device_;         
    openni::VideoStream ir_;      
    openni::VideoStream color_;    
    openni::Status rc_;
    openni::VideoFrameRef irf_;
    openni::VideoFrameRef colorf_;
    OpenNI2Grabber <PointType> *openni2_grabber_ = 0;
    pcl::PointCloud<PointType>::Ptr cloud_ ;
    int mode_;
    int frame_count_;

    OpenniStreamer ();

    OpenniStreamer (std::string filename);

    pcl::PointCloud<PointType>::Ptr GetCloud ();

    bool HasDataLeft ();
    int GetFrameIndex();
};

#endif