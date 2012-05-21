#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ObjectCapture
{
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointNormal PointNormalT;
    typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
}
