#include "KeyDes.hpp"
#include <pcl/features/3dsc.h>
#include <pcl/features/usc.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>
#include <pcl/point_types_conversion.h>


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

    ShapeContext3D (PCP model_keypoints, PCP scene_keypoints, PCNP model_normals, PCNP scene_normals);

    pcl::CorrespondencesPtr
    run();

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
    pcl::UniqueShapeContext<PointType, SHAPE, RFType>  usc_;
    bool created_;
    PCP model_keypoints_;
    PCP scene_keypoints_;
    PCSP model_descriptors_;
    PCSP scene_descriptors_;

    Usc (PCP model_keypoints, PCP scene_keypoints);

    pcl::CorrespondencesPtr
    run();

};
/*
class Spin
{
public:
    typedef pcl::Histogram<153> SPIN;
    typedef pcl::search::KdTree<PointType> KDT;
    typedef KDT::Ptr KDTP;
    typedef pcl::PointCloud<PointType>::Ptr PCP;
    typedef pcl::PointCloud<NormalType>::Ptr PCNP;
    typedef pcl::PointCloud<SPIN> PCS;
    typedef PCS::Ptr PCSP;
    bool created_;
    pcl::SpinImageEstimation<PointType, NormalType, SPIN> si_;
    KDTP kdtree_;
    PCP model_keypoints_;
    PCP scene_keypoints_;
    PCNP model_normals_;
    PCNP scene_normals_;
    PCSP model_descriptors_;
    PCSP scene_descriptors_;

    Spin (PCP model_keypoints, PCP scene_keypoints, PCNP model_normals, PCNP scene_normals);

    pcl::CorrespondencesPtr
    run();
};

class Rift
{
public:
    typedef pcl::Histogram<32> RIFT;
    typedef pcl::PointXYZI XYZIType;
    typedef pcl::PointCloud<XYZIType> PCI;
    typedef pcl::search::KdTree<XYZIType> KDT;
    typedef pcl::IntensityGradient IGType;
    typedef pcl::PointCloud<IGType> PCIG;
    typedef KDT::Ptr KDTP;
    typedef pcl::PointCloud<PointType>::Ptr PCP;
    typedef pcl::PointCloud<NormalType>::Ptr PCNP;
    typedef pcl::PointCloud<RIFT> PCR;
    typedef PCR::Ptr PCRP;
    bool created_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntensity_;
    pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients_;
    pcl::IntensityGradientEstimation <XYZIType, NormalType, IGType, pcl::common::IntensityFieldAccessor<XYZIType>> ge_;
    pcl::RIFTEstimation<XYZIType, IGType, RIFT> rift_;
    KDTP kdtree_;
    PCP model_keypoints_;
    PCP scene_keypoints_;
    PCNP model_normals_;
    PCNP scene_normals_;
    PCRP model_descriptors_;
    PCRP scene_descriptors_;

    Rift (PCP model_keypoints, PCP scene_keypoints, PCNP model_normals, PCNP scene_normals);

    pcl::CorrespondencesPtr
    run();
};
*/