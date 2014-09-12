#include "KeyDes.hpp"
#include <pcl/features/3dsc.h>
#include <pcl/features/usc.h>


class ShapeContext3D {
public:
	pcl::ShapeContext3DEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::ShapeContext1980> sc3d_;
	bool created_;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree_;
	pcl::PointCloud<PointType>::Ptr model_keypoints_;
	pcl::PointCloud<PointType>::Ptr scene_keypoints_;
	pcl::PointCloud<NormalType>::Ptr model_normals_;
	pcl::PointCloud<NormalType>::Ptr scene_normals_;
	pcl::PointCloud<pcl::ShapeContext1980>::Ptr model_descriptors_;
	pcl::PointCloud<pcl::ShapeContext1980>::Ptr scene_descriptors_;

	ShapeContext3D (pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<PointType>::Ptr scene_keypoints, pcl::PointCloud<NormalType>::Ptr model_normals,
	                pcl::PointCloud<NormalType>::Ptr scene_normals) :
    model_descriptors_ (new pcl::PointCloud<pcl::ShapeContext1980> ()), scene_descriptors_ (new pcl::PointCloud<pcl::ShapeContext1980> ()), model_keypoints_ (model_keypoints),
    scene_keypoints_ (scene_keypoints), model_normals_ (model_normals), scene_normals_ (scene_normals), created_ (false), kdtree_(new pcl::search::KdTree<PointType>)
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

		pcl::KdTreeFLANN<pcl::ShapeContext1980> match_search;

		//  Find Model-Scene Correspondences with KdTree
		std::cout << "calculating correspondences " << std::endl;

		match_search.setInputCloud (model_descriptors_);

		//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	    #pragma omp parallel for 
	    for (size_t i = 0; i < scene_descriptors_->size (); ++i)
	    {
			std::vector<int> neigh_indices (1);
			std::vector<float> neigh_sqr_dists (1);
			if (match_search.point_representation_->isValid (scene_descriptors_->at (i)))
			{
				int found_neighs = match_search.nearestKSearch (scene_descriptors_->at (i), 1, neigh_indices, neigh_sqr_dists);
				if (found_neighs == 1 && neigh_sqr_dists[0] < descriptor_distance)
				{
					pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
					#pragma omp critical
					model_scene_corrs->push_back (corr);
				}
			}
	    }

		std::cout << "\tFound " << model_scene_corrs->size () << " correspondences " << std::endl;
		return (model_scene_corrs);
	}

};

class Usc {
public:
	pcl::UniqueShapeContext<pcl::PointXYZRGB, pcl::ShapeContext1980, pcl::ReferenceFrame>  usc_;
	bool created_;
	pcl::PointCloud<PointType>::Ptr model_keypoints_;
	pcl::PointCloud<PointType>::Ptr scene_keypoints_;
	pcl::PointCloud<pcl::ShapeContext1980>::Ptr model_descriptors_;
	pcl::PointCloud<pcl::ShapeContext1980>::Ptr scene_descriptors_;

	Usc (pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<PointType>::Ptr scene_keypoints):
    model_descriptors_ (new pcl::PointCloud<pcl::ShapeContext1980> ()), scene_descriptors_ (new pcl::PointCloud<pcl::ShapeContext1980> ()), model_keypoints_ (model_keypoints),
    scene_keypoints_ (scene_keypoints), created_ (false)
    {
    	usc_.setRadiusSearch(0.05);
    	usc_.setMinimalRadius(0.05 / 10.0);
    	usc_.setPointDensityRadius(0.05 / 5.0);
    	usc_.setLocalRadius(0.05);
    }

    pcl::CorrespondencesPtr
    run()
    {
    	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

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

		pcl::KdTreeFLANN<pcl::ShapeContext1980> match_search;

		//  Find Model-Scene Correspondences with KdTree
		std::cout << "calculating correspondences " << std::endl;

		match_search.setInputCloud (model_descriptors_);

		//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	    #pragma omp parallel for 
	    for (size_t i = 0; i < scene_descriptors_->size (); ++i)
	    {
			std::vector<int> neigh_indices (1);
			std::vector<float> neigh_sqr_dists (1);
			if (match_search.point_representation_->isValid (scene_descriptors_->at (i)))
			{
				int found_neighs = match_search.nearestKSearch (scene_descriptors_->at (i), 1, neigh_indices, neigh_sqr_dists);
				if (found_neighs == 1 && neigh_sqr_dists[0] < descriptor_distance)
				{
					pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
					#pragma omp critical
					model_scene_corrs->push_back (corr);
				}
			}
	    }

		std::cout << "\tFound " << model_scene_corrs->size () << " correspondences " << std::endl;
		return (model_scene_corrs);
	}

};