#include "OpenniStreamer.h"

OpenniStreamer::OpenniStreamer ()
{
  mode_ = 0;
  frame_count_ = 0;
  cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  rc_ = openni::OpenNI::initialize ();  
  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "OpenNI initialization failed" << std::endl;
    openni::OpenNI::shutdown ();
  }
  else
    std::cout << "OpenNI initialization successful" << std::endl;

  openni::Array<openni::DeviceInfo> deviceList;
  openni::OpenNI::enumerateDevices(&deviceList);

  for(int i = 0; i< deviceList.getSize(); ++i)
    std::cout << deviceList[i].getUri() <<std::endl;

  if( device_num >= 0 && device_num < deviceList.getSize() )
    rc_ = device_.open(deviceList[device_num].getUri());
  else {
    std::cout << "Wrong device identifier" << std::endl;
    rc_ = device_.open(openni::ANY_DEVICE);
  }

  openni2_grabber_ = new OpenNI2Grabber < pcl::PointXYZRGB > (deviceList[0].getUri());

  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Device initialization failed" << std::endl;
    device_.close ();
  }
  rc_ = ir_.create (device_, openni::SENSOR_DEPTH);   
  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Ir sensor creation failed" << std::endl;
    ir_.destroy ();
  }
  else
    std::cout << "Ir sensor creation successful" << std::endl;
  rc_ = ir_.start ();                      
  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Ir activation failed" << std::endl;
    ir_.destroy ();
  }
  else
    std::cout << "Ir activation successful" << std::endl;

  device_.setImageRegistrationMode (openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

  rc_ = color_.create (device_, openni::SENSOR_COLOR);    

  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Color sensor creation failed" << std::endl;
    color_.destroy ();
  }
  else
    std::cout << "Color sensor creation successful" << std::endl;
  rc_ = color_.start ();                   

  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Color sensor activation failed" << std::endl;
    color_.destroy ();
  }
  else
    std::cout << "Color sensor activation successful" << std::endl;
}

OpenniStreamer::OpenniStreamer (std::string file_name)
{
  mode_ = 1;
  frame_count_ = 0;
  cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  rc_ = openni::OpenNI::initialize (); 
  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "OpenNI initialization failed" << std::endl;
    openni::OpenNI::shutdown ();
  }
  else
    std::cout << "OpenNI initialization successful" << std::endl;
  const char *cstr = file_name.c_str();
  rc_ = device_.open(cstr);
  device_.trigger();
  openni2_grabber_ = new OpenNI2Grabber < pcl::PointXYZRGB > ("freenect");

  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Device initialization failed" << std::endl;
    device_.close ();
  }
  rc_ = ir_.create (device_, openni::SENSOR_DEPTH);    
  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Ir sensor creation failed" << std::endl;
    ir_.destroy ();
  }
  else
    std::cout << "Ir sensor creation successful" << std::endl;
  rc_ = ir_.start ();                     
  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Ir activation failed" << std::endl;
    ir_.destroy ();
  }
  else
    std::cout << "Ir activation successful" << std::endl;

  device_.setImageRegistrationMode (openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

  rc_ = color_.create (device_, openni::SENSOR_COLOR);    

  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Color sensor creation failed" << std::endl;
    color_.destroy ();
  }
  else
    std::cout << "Color sensor creation successful" << std::endl;
  rc_ = color_.start ();                      

  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Color sensor activation failed" << std::endl;
    color_.destroy ();
  }
  else
    std::cout << "Color sensor activation successful" << std::endl;
}

pcl::PointCloud<PointType>::Ptr OpenniStreamer::GetCloud(){
  ir_.readFrame (&irf_);
  color_.readFrame (&colorf_);
  if(mode_ == 0){
    cloud_ = openni2_grabber_->get_point_cloud ((openni::RGB888Pixel*)colorf_.getData(), (uint16_t*)irf_.getData(), distance, true, colorf_.getHeight(), colorf_.getWidth(), reg, false);
  }else{
    cloud_ = openni2_grabber_->get_point_cloud ((openni::RGB888Pixel*)colorf_.getData(), (uint16_t*)irf_.getData(), distance, true, colorf_.getHeight(), colorf_.getWidth(), reg, true);
  }
  frame_count_++;
  return cloud_;
}

bool OpenniStreamer::HasDataLeft(){
  if (mode_ == 0)
    return true;
  else
    return (device_.getPlaybackControl()->getNumberOfFrames(ir_) >= frame_count_);//device_.hasDataLeft ();
}

int OpenniStreamer::GetFrameIndex(){
  return irf_.getFrameIndex();
}