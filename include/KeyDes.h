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
    pcl::UniqueShapeContext<PointType, SHAPE, RFType>  usc_;
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

    Spin (PCP model_keypoints, PCP scene_keypoints, PCNP model_normals, PCNP scene_normals) :
          model_descriptors_ (new PCS ()), scene_descriptors_ (new PCS()), model_keypoints_ (model_keypoints),
          scene_keypoints_ (scene_keypoints), model_normals_ (model_normals), scene_normals_ (scene_normals),
          created_ (false), kdtree_(new KDT())
    {
      si_.setRadiusSearch(0.02);
      si_.setImageWidth(8);
    }

    pcl::CorrespondencesPtr
    run()
    {
      pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

      if (!created_)
      {
        //create model descriptors
        std::cout << "calculating model descriptors " << std::endl;
        si_.setInputCloud (model_keypoints_);
        si_.setInputNormals (model_normals_);
        si_.compute (*model_descriptors_);
        created_ = true;
      }
      si_.setInputCloud(scene_keypoints_);
      si_.setInputNormals(scene_normals_);
      si_.compute(*scene_descriptors_);

      return (MatchDescriptors<SPIN>(scene_descriptors_, model_descriptors_));
    }
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

    Rift (PCP model_keypoints, PCP scene_keypoints, PCNP model_normals, PCNP scene_normals) :
          model_descriptors_ (new PCR), scene_descriptors_ (new PCR), model_keypoints_ (model_keypoints),
          scene_keypoints_ (scene_keypoints), model_normals_ (model_normals), scene_normals_ (scene_normals),
          created_ (false), kdtree_(new KDT), cloudIntensity_(new PCI), gradients_(new PCIG)

    {
      ge_.setRadiusSearch(0.03);
      rift_.setSearchMethod(kdtree_);
      rift_.setRadiusSearch(0.02);
      // Set the number of bins to use in the distance dimension.
      rift_.setNrDistanceBins(4);
      // Set the number of bins to use in the gradient orientation dimension.
      rift_.setNrGradientBins(8);
    }

    pcl::CorrespondencesPtr
    run()
    {
      pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());


      if (!created_)
      {
        pcl::PointCloudXYZRGBtoXYZI(*model_keypoints_, *cloudIntensity_);
        //create model descriptors
        std::cout << "calculating model descriptors " << std::endl;
        ge_.setInputCloud(cloudIntensity_);
        ge_.setInputNormals(model_normals_);
        ge_.compute(*gradients_);
        rift_.setInputCloud(cloudIntensity_);
        rift_.setInputGradient(gradients_);
        rift_.compute(*model_descriptors_);
        created_ = true;
      }
      pcl::PointCloudXYZRGBtoXYZI(*scene_keypoints_, *cloudIntensity_);
      ge_.setInputCloud(cloudIntensity_);
      ge_.setInputNormals(scene_normals_);
      ge_.compute(*gradients_);
      rift_.setInputCloud(cloudIntensity_);
      rift_.setInputGradient(gradients_);
      rift_.compute(*scene_descriptors_);

      return (MatchDescriptors<RIFT>(scene_descriptors_, model_descriptors_));
    }
};
