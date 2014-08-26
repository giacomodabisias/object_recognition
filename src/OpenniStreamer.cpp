#include "OpenniStreamer.h"

OpenniStreamer::OpenniStreamer ()
{
  mode_ = 0;
  cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  rc_ = openni::OpenNI::initialize ();  // Initialize OpenNI 
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
  rc_ = ir_.create (device_, openni::SENSOR_DEPTH);    // Create the VideoStream for IR

  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Ir sensor creation failed" << std::endl;
    ir_.destroy ();
  }
  else
    std::cout << "Ir sensor creation successful" << std::endl;
  rc_ = ir_.start ();                      // Start the IR VideoStream
  //ir.setMirroringEnabled(TRUE); 
  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Ir activation failed" << std::endl;
    ir_.destroy ();
  }
  else
    std::cout << "Ir activation successful" << std::endl;

  device_.setImageRegistrationMode (openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

  //ir.setImageRegistrationMode(ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR);
  rc_ = color_.create (device_, openni::SENSOR_COLOR);    // Create the VideoStream for Color

  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Color sensor creation failed" << std::endl;
    color_.destroy ();
  }
  else
    std::cout << "Color sensor creation successful" << std::endl;
  rc_ = color_.start ();                      // Start the Color VideoStream

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
  cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  rc_ = openni::OpenNI::initialize ();  // Initialize OpenNI 
  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "OpenNI initialization failed" << std::endl;
    openni::OpenNI::shutdown ();
  }
  else
    std::cout << "OpenNI initialization successful" << std::endl;
  const char *cstr = file_name.c_str();
  rc_ = device_.open(cstr);

 
  openni2_grabber_ = new OpenNI2Grabber < pcl::PointXYZRGB > ("freenect");

  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Device initialization failed" << std::endl;
    device_.close ();
  }
  rc_ = ir_.create (device_, openni::SENSOR_DEPTH);    // Create the VideoStream for IR

  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Ir sensor creation failed" << std::endl;
    ir_.destroy ();
  }
  else
    std::cout << "Ir sensor creation successful" << std::endl;
  rc_ = ir_.start ();                      // Start the IR VideoStream
  //ir.setMirroringEnabled(TRUE); 
  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Ir activation failed" << std::endl;
    ir_.destroy ();
  }
  else
    std::cout << "Ir activation successful" << std::endl;

  device_.setImageRegistrationMode (openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

  //ir.setImageRegistrationMode(ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR);
  rc_ = color_.create (device_, openni::SENSOR_COLOR);    // Create the VideoStream for Color

  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Color sensor creation failed" << std::endl;
    color_.destroy ();
  }
  else
    std::cout << "Color sensor creation successful" << std::endl;
  rc_ = color_.start ();                      // Start the Color VideoStream

  if (rc_ != openni::STATUS_OK)
  {
    std::cout << "Color sensor activation failed" << std::endl;
    color_.destroy ();
  }
  else
    std::cout << "Color sensor activation successful" << std::endl;
}
/*
OpenniStreamer::OpenniStreamer(std::string filename){
  cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  mode_ = 1;
  grabber_ = new pcl::ONIGrabber (oni_file, false, false);
  boost::function<void (const CloudConstPtr&) > f = boost::bind (&OpenniStreamer::Cloud_cb, this, _1);
  boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) > f_1 = boost::bind (&OpenniStreamer::Depth_cb, this, _1);
  boost::function<void (const boost::shared_ptr<openni_wrapper::Image> &image) > f_2 = boost::bind (&OpenniStreamer::Image_cb, this, _1 );
  
  c_ = grabber_->registerCallback (f);
  c_ = grabber_->registerCallback (f_1);
  c_ = grabber_->registerCallback (f_2);
  pass_.setFilterFieldName ("z");
  pass_.setFilterLimits (0.0, distance/500); //distance in meters
}

void OpenniStreamer::Image_cb (const boost::shared_ptr<openni_wrapper::Image> &image){
  if (image_read_ == false){
    image_ = image;
    image_read_ = true;
  }
}

void OpenniStreamer::Depth_cb (const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image){
  if (depth_read_ == false){
    depth_image_ = depth_image;
    depth_read_ = true;
  }
}

void OpenniStreamer::Cloud_cb (const CloudConstPtr& cloud){
  if (cloud_read_ == false){
    pass_.setInputCloud (cloud);
    pass_.filter (*cloud_);
    cloud_read_ = true;
  }
}

pcl::PointCloud<PointType>::Ptr OpenniStreamer::GetCloud(){
  if(mode_ == 0){
    ir_.readFrame (&irf_);
    color_.readFrame (&colorf_);
    cloud_ = openni2_grabber_.get_point_cloud (colorf_, irf_, distance, true);
  }else{
    cloud_read_ = false;
    while(cloud_read_ == false){
      grabber_->start();
    }
  }
  return cloud_;
}


pcl::PointCloud<PointType>::Ptr OpenniStreamer::GetCloud(){
  if(mode_ == 0){
    ir_.readFrame (&irf_);
    color_.readFrame (&colorf_);
    cloud_ = openni2_grabber_->get_point_cloud ((openni::RGB888Pixel*)colorf_.getData(), (uint16_t*)irf_.getData(), distance, true, colorf_.getHeight(), colorf_.getWidth(), false, false);
  }else{
    image_read_ = false;
    depth_read_ = false;
    while(image_read_ == false || depth_read_ == false){
      grabber_->start();
    }
    cloud_ = openni2_grabber_->get_point_cloud ((openni::RGB888Pixel*)(image_->getMetaData().Data()), (uint16_t*)(depth_image_->getDepthMetaData().Data()), distance, true, 480, 640, true, true);
  }
  return cloud_;
}
*/

pcl::PointCloud<PointType>::Ptr OpenniStreamer::GetCloud(){
  ir_.readFrame (&irf_);
  color_.readFrame (&colorf_);
  if(mode_ == 0){
    cloud_ = openni2_grabber_->get_point_cloud ((openni::RGB888Pixel*)colorf_.getData(), (uint16_t*)irf_.getData(), distance, true, colorf_.getHeight(), colorf_.getWidth(), false, false);
  }else{
    cloud_ = openni2_grabber_->get_point_cloud ((openni::RGB888Pixel*)colorf_.getData(), (uint16_t*)irf_.getData(), distance, true, colorf_.getHeight(), colorf_.getWidth(), true, true);
  }
  return cloud_;
}

bool OpenniStreamer::HasDataLeft(){
  if (mode_ == 0)
    return true;
  else
    return true;//device_.hasDataLeft ();
}
