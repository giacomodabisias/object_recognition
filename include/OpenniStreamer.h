#ifndef OPENNI_STREAMER_H
#define OPENNI_STREAMER_H

#include <pcl/filters/passthrough.h>
#include "openni2_grabber.hpp"
#include "pcl/io/oni_grabber.h"
#include "define.h"
#include "OpenNI.h"


class OpenniStreamer
{
  public:
    openni::Device device_;         
    openni::VideoStream ir_;      
    openni::VideoStream color_;    
    openni::Status rc_;
    boost::signals2::connection c_;
    pcl::ONIGrabber* grabber_;
    openni::VideoFrameRef irf_;
    openni::VideoFrameRef colorf_;
    OpenNI2Grabber < pcl::PointXYZRGB > *openni2_grabber_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ ;
    int mode_;
    bool cloud_read_ = false;
    bool image_read_ = false;
    bool depth_read_ = false;
    boost::shared_ptr<openni_wrapper::Image> image_;
    boost::shared_ptr<openni_wrapper::DepthImage> depth_image_;

    pcl::PassThrough<pcl::PointXYZRGB> pass_;
    
    OpenniStreamer ();

    OpenniStreamer(std::string filename);

    pcl::PointCloud<PointType>::Ptr GetCloud();

    bool HasDataLeft();
};

#endif