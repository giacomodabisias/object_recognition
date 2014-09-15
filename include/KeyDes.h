#include "KeyDes.hpp"
#include <pcl/features/3dsc.h>
#include <pcl/features/usc.h>


class ShapeContext3D {
public:
    typedef pcl::ShapeContext1980 SHAPE;
    typedef pcl::search::KdTree<PointType> KDT;
    typedef KDT::Ptr KDTP;
    typedef pcl::PointCloud<PointType>::Ptr PCP;
    typedef pcl::PointCloud<NormalType>::Ptr PCNP;
    typedef pcl::PointCloud<SHAPE> PCS;
    typedef PCS::Ptr PCSP;
    bool created_;
    pcl::ShapeContext3DEstimation<PointType, NormalType, SHAPE> sc3d_;
    KDTP kdtree_;
    PCP model_keypoints_;
    PCP scene_keypoints_;
    PCNP model_normals_;
    PCNP scene_normals_;
    PCSP model_descriptors_;
    PCSP scene_descriptors_;

    ShapeContext3D (PCP model_keypoints, PCP scene_keypoints, PCNP model_normals, PCNP scene_normals) :
                    model_descriptors_ (new PCS ()), scene_descriptors_ (new PCS()), model_keypoints_ (model_keypoints),
                    scene_keypoints_ (scene_keypoints), model_normals_ (model_normals), scene_normals_ (scene_normals),
                    created_ (false), kdtree_(new KDT())
    {
        sc3d_.setSearchMethod(kdtree_);
        sc3d_.setRadiusSearch(0.05);
        sc3d_.setMinimalRadius(0.05 / 10.0);
        sc3d_.setPointDensityRadius(0.05 / 5.0);

    }

    pcl::CorrespondencesPtr
    run()
    {
        pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

        if (!created_)
        {
            //create model descriptors
            std::cout << "calculating model descriptors " << std::endl;
            sc3d_.setInputCloud (model_keypoints_);
            sc3d_.setInputNormals (model_normals_);
            sc3d_.compute (*model_descriptors_);
            created_ = true;
        }
        sc3d_.setInputCloud(scene_keypoints_);
        sc3d_.setInputNormals(scene_normals_);
        sc3d_.compute(*scene_descriptors_);

        return (MatchDescriptors<SHAPE>(scene_descriptors_, model_descriptors_));
    }

};

class Usc {
public:
    typedef pcl::ShapeContext1980 SHAPE;
    typedef pcl::search::KdTree<PointType> KDT;
    typedef KDT::Ptr KDTP;
    typedef pcl::PointCloud<PointType>::Ptr PCP;
    typedef pcl::PointCloud<NormalType>::Ptr PCNP;
    typedef pcl::PointCloud<SHAPE> PCS;
    typedef PCS::Ptr PCSP;
    pcl::UniqueShapeContext<PointType, SHAPE, pcl::ReferenceFrame>  usc_;
    bool created_;
    PCP model_keypoints_;
    PCP scene_keypoints_;
    PCSP model_descriptors_;
    PCSP scene_descriptors_;

    Usc (PCP model_keypoints, PCP scene_keypoints):
    model_descriptors_ (new PCS ()), scene_descriptors_ (new PCS ()), model_keypoints_ (model_keypoints), scene_keypoints_ (scene_keypoints), created_ (false)
    {
        usc_.setRadiusSearch(0.05);
        usc_.setMinimalRadius(0.05 / 10.0);
        usc_.setPointDensityRadius(0.05 / 5.0);
        usc_.setLocalRadius(0.05);
    }

    pcl::CorrespondencesPtr
    run()
    {
        if (!created_)
        {
            //create model descriptors
            std::cout << "calculating model descriptors " << std::endl;
            usc_.setInputCloud (model_keypoints_);
            usc_.compute (*model_descriptors_);
            created_ = true;
        }
        usc_.setInputCloud(scene_keypoints_);
        usc_.compute(*scene_descriptors_);
        
        return (MatchDescriptors<SHAPE>(scene_descriptors_, model_descriptors_));
    }

};

