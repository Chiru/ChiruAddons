
// For conditions of distribution and use, see copyright notice in LICENSE

#include "StableHeaders.h"
#include "DebugOperatorNew.h"
#include "LoggingFunctions.h"

#include "CloudFilter.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <QString>

#include "MemoryLeakCheck.h"

namespace ObjectCapture
{

CloudFilter::CloudFilter()
{
    passthrough_filter_.setFilterFieldName("z");
    sac_segmentation_.setOptimizeCoefficients (true);
    // Mandatory
    sac_segmentation_.setModelType (pcl::SACMODEL_PLANE);
    sac_segmentation_.setMethodType (pcl::SAC_RANSAC);
    sac_segmentation_.setDistanceThreshold (0.02);
    cluster_extractor_.setMinClusterSize (1000);
    cluster_extractor_.setMaxClusterSize (250000);
}

CloudFilter::~CloudFilter()
{

}

PointCloud::Ptr CloudFilter::filterDepth(PointCloud::ConstPtr cloud, float minDepth, float maxDepth)
{
    PointCloud::Ptr output(new PointCloud);
    if(cloud->points.size() > 0)
    {
        passthrough_filter_.setFilterLimits(minDepth, maxDepth);
        passthrough_filter_.setInputCloud(cloud);
        passthrough_filter_.filter(*output);
        LogDebug("ObjectCapture: Depth filtered cloud from " + QString::number(cloud->points.size()) + " points to " + QString::number(output->points.size()) + " points.");
    }
    return output;
}

PointCloud::Ptr CloudFilter::filterDensity(PointCloud::ConstPtr cloud, float leafSize)
{
    PointCloud::Ptr output(new PointCloud);
    pcl::PointCloud<int> indices;
    if (cloud->points.size() > 0)
    {
        uniform_sampling_.setRadiusSearch(leafSize);
        uniform_sampling_.setInputCloud(cloud);
        uniform_sampling_.compute(indices);
    }
    pcl::copyPointCloud(*cloud, indices.points, *output);
    LogDebug("ObjectCapture: Density filtered cloud from " + QString::number(cloud->points.size()) + " points to " + QString::number(output->points.size()) + " points.");
    return output;
}

PointCloud::Ptr CloudFilter::removePlanar(PointCloud::ConstPtr cloud)
{
    PointCloud::Ptr output(new PointCloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    if(cloud->points.size() > 0)
    {
        sac_segmentation_.setInputCloud(cloud);
        sac_segmentation_.segment(*inliers, *coefficients);
        indice_extractor_.setInputCloud(cloud);
        indice_extractor_.setIndices(inliers);
        indice_extractor_.setNegative(true);
        indice_extractor_.filter(*output);
        LogDebug("ObjectCapture: Removed planar of " + QString::number(cloud->points.size() - output->points.size()) + " points from the cloud.");
    }
    return output;
}

PointCloud::Ptr CloudFilter::extractLargestCluster(PointCloud::ConstPtr cloud, float cluster_tolerance)
{
    PointCloud::Ptr output(new PointCloud);
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    std::vector<pcl::PointIndices> cluster_indices;

    if (cloud->points.size() > 0)
    {
        tree->setInputCloud (cloud);
        cluster_extractor_.setClusterTolerance(cluster_tolerance);
        cluster_extractor_.setSearchMethod(tree);
        cluster_extractor_.setInputCloud(cloud);
        cluster_extractor_.extract(cluster_indices);

        if(cluster_indices.size() > 0)
        {
            pcl::IndicesPtr indices (new std::vector<int>);
            *indices = cluster_indices[0].indices;
            indice_extractor_.setInputCloud(cloud);
            indice_extractor_.setIndices(indices);
            indice_extractor_.setNegative(false);
            indice_extractor_.filter(*output);
            LogDebug("ObjectCapture: Extracted largest cluster of " + QString::number(output->points.size()) + " points from the cloud.");
            return output;
        }
    }

    LogError("ObjectCapture: Cluster extraction failed, no clusters found.");
    pcl::copyPointCloud(*cloud, *output);
    return output;
}

PointCloud::Ptr CloudFilter::segmentCloud(PointCloud::ConstPtr cloud, float cluster_tolerance)
{
    if(cloud->points.size() > 0)
        return extractLargestCluster(removePlanar(cloud), cluster_tolerance);
    else
        return PointCloud::Ptr(new PointCloud);
}

}
