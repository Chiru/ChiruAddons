// For conditions of distribution and use, see copyright notice in LICENSE

#include "Registering.h"
#include "CloudFilter.h"

#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>

#include "LoggingFunctions.h"

#include <pcl/io/file_io.h>

namespace ObjectCapture
{

IncrementalRegister::IncrementalRegister() :
    cloud_filter_(new CloudFilter),
    icp_corr_distance_(0.07f),
    ransac_outlier_threshold_(0.05f),
    uniform_leafsize_(0.01f)
{
}

IncrementalRegister::~IncrementalRegister()
{
}

void IncrementalRegister::registerCloud(PointCloud::ConstPtr cloud)
{
    if(final_clouds_.size() == 0)
    {
        addCloudToFinalModel(cloud, Eigen::Matrix4f::Identity());
    }
    else
    {
        Eigen::Matrix4f transformation;

        // Apply initial alignment (i.e. the alignment of the latest registered cloud).
        PointCloud::Ptr initial_aligned(new PointCloud);
        pcl::transformPointCloud(*cloud, *initial_aligned, getGlobalTransformation());

        registerPair(getCurrentCloud(), initial_aligned, transformation);

        addTransformation(transformation);
        addCloudToFinalModel(initial_aligned, transformation);
    }
    emit globalModelUpdated(getCurrentCloud());
}

bool IncrementalRegister::removeLatestCloud()
{
    if(final_clouds_.size() <= 0)
        return false;

    removeLatestTransformation();
    final_clouds_.erase(final_clouds_.end());
    emit globalModelUpdated(getCurrentCloud());
    return true;
}

void IncrementalRegister::reset()
{
    transformations_.clear();
    std::vector<PointCloud::Ptr>::iterator cloud_it;
    for(cloud_it = final_clouds_.begin(); cloud_it != final_clouds_.end(); ++cloud_it)
    {
        (*cloud_it).reset();
    }
    final_clouds_.clear();
}

PointCloud::Ptr IncrementalRegister::getCurrentCloud()
{
    if(final_clouds_.size() <= 0)
    {
        return PointCloud::Ptr(new PointCloud);
    }
    else if(final_clouds_.size() == 1)
    {
        return final_clouds_.at(0);
    }
    else
    {
        PointCloud::Ptr full_cloud(new PointCloud);
        std::vector<PointCloud::Ptr>::const_iterator it;
        for(it = final_clouds_.begin(); it != final_clouds_.end(); ++it)
        {
            *full_cloud += *(*it);
        }
        return cloud_filter_->filterDensity(full_cloud, uniform_leafsize_);
    }
}

void IncrementalRegister::registerPair(PointCloud::Ptr src, PointCloud::Ptr tgt, Eigen::Matrix4f &pair_transformation)
{
    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    CurvaturePointRepresentation point_representation;
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);

    pcl::IterativeClosestPoint<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-6);

    reg.setMaxCorrespondenceDistance (icp_corr_distance_); // 0.3 w
    reg.setRANSACOutlierRejectionThreshold(ransac_outlier_threshold_); // 0.2 w

    reg.setPointRepresentation (boost::make_shared<const CurvaturePointRepresentation> (point_representation));

    reg.setInputCloud (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (2);
    for (int i = 0; i < 60; ++i)
    {
      points_with_normals_src = reg_result;

      // Estimate
      reg.setInputCloud (points_with_normals_src);
      reg.align (*reg_result);

      // Accumulate transformation between each Iteration
      Ti = reg.getFinalTransformation () * Ti;

      // Refine the process
      if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ()) {
          //PCL_INFO("Decreasing maxium correspondence distance.\n");
          reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.005); // 0.001
      }

      prev = reg.getLastIncrementalTransformation ();

      //PCL_INFO ("Iteration Nr. %d. (%f)\n", i, reg.getFitnessScore());

      if(reg.getFitnessScore() < 0.0001f) {
          //PCL_INFO("Convergence reached, breaking early.\n");
          break;
      }
    }

    pair_transformation = Ti.inverse();
}

void IncrementalRegister::addCloudToFinalModel(PointCloud::ConstPtr cloud, Eigen::Matrix4f transformation)
{
    PointCloud::Ptr transformed_cloud(new PointCloud);
    // Transform cloud to global coordinate system
    pcl::transformPointCloud(*cloud, *transformed_cloud, transformation);
    final_clouds_.push_back(transformed_cloud);
}

void IncrementalRegister::addTransformation(Eigen::Matrix4f &transformation)
{
    transformations_.push_back(transformation);
}

void IncrementalRegister::removeLatestTransformation()
{
    if(transformations_.size() > 0)
        transformations_.erase(transformations_.end());
}

Eigen::Matrix4f IncrementalRegister::getGlobalTransformation()
{
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >::const_iterator it;
    for(it = transformations_.begin(); it != transformations_.end(); ++it)
    {
        transformation = (*it) * transformation;
    }
    return transformation;
}

}
