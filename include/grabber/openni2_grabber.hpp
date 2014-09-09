#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "OpenNI.h"
#include <stdio.h>
#include <iostream>
#include "registration.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> 
class OpenNI2Grabber
{
    public:

        uint16_t * depth_maptmp = 0;
        bool init = true;
        KinectRegistration *reg = 0;
        std::string dev_type_;

    OpenNI2Grabber(std::string dev_type):dev_type_(dev_type){}

    typename pcl::PointCloud<PointT>::Ptr 
    get_point_cloud(openni::RGB888Pixel* rgb_buffer, uint16_t* depth_map, int distance, bool colored, int image_height, int image_width, bool kreg, bool set_mirror) 
    {

        int depth_height = image_height;
        int depth_width = image_width;

        if(kreg)
        {   
            if(init){
                reg = new KinectRegistration(false);
                reg->init(depth_width, depth_height, 1, 1);
                if(set_mirror)
                    reg->SetMirror();
                init = !init;
                depth_maptmp = (uint16_t*)malloc(sizeof(uint16_t)*depth_width*depth_height);
            }
            reg->apply_registration(depth_map, depth_maptmp);
            depth_map = depth_maptmp;
        }

        //create the empty Pointcloud
        boost::shared_ptr<pcl::PointCloud<PointT>> cloud (new pcl::PointCloud<PointT>);
        
        //allow infinite values for points coordinates
        cloud->is_dense = false;
        double focal_x_depth = 5.9421434211923247e+02;
        double focal_y_depth = 5.9104053696870778e+02;
        double center_x_depth = 3.3930780975300314e+02;
        double center_y_depth = 2.4273913761751615e+02;
        //set camera parameters for kinect
        if(dev_type_.find("freenect")!= -1){
            //set camera parameters for kinect
            double focal_x_depth = 5.9421434211923247e+02;
            double focal_y_depth = 5.9104053696870778e+02;
            double center_x_depth = 3.3930780975300314e+02;
            double center_y_depth = 2.4273913761751615e+02;
        }else{
            //set camera parameters for asus
            double focal_x_depth = 585.187492217609;
            double focal_y_depth = 585.308616340665;
            double center_x_depth = 322.714077555293;
            double center_y_depth = 248.626108676666;
        }

        //fill the cloud using the rgb and depth buffers
        for (unsigned int y = 0; y < depth_height; ++y)
            for (unsigned int x = 0; x < depth_width; ++x){
                PointT ptout;
                uint16_t dz = depth_map[y*depth_width + x];
                if (abs(dz) < distance){
                    // find the correct rgb point which corresponds to the depth point
                    Eigen::Vector3d ptd((x - center_x_depth) * dz / focal_x_depth, (y - center_y_depth) * dz/focal_y_depth, dz);
                    // assign output xyz
                    if(ptd.x() == 0 && ptd.y() == 0 && ptd.z() == 0){
                        continue;
                    }else{
                        ptout.x = ptd.x()*0.001f;
                        ptout.y = ptd.y()*0.001f;
                        ptout.z = ptd.z()*0.001f;
                    }
                    if(colored){
                        openni::RGB888Pixel pix = rgb_buffer[y*depth_width + x];
                        ptout.rgba = pcl::PointXYZRGB(pix.r,pix.g,pix.b).rgba; //assign color 
                    } else{
                        ptout.rgba = pcl::PointXYZRGB(0, 0, 0).rgba;
                    }
                    cloud->points.push_back(ptout); //assigns point to cloud   
                } 
                
            }

        cloud->height = depth_height;
        cloud->width = depth_width;
        return (cloud);
    }
};