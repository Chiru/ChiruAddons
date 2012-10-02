// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "ObjectCaptureModuleDefines.h"

#include <pcl/point_representation.h>
#include <Eigen/StdVector>
#include <Eigen/Core>

#include <QObject>
#include <QList>
#include <QString>
#include <QMutex>

namespace ObjectCapture
{
class CloudFilter;

/// Custom point presentation for points with pointnormal
class CurvaturePointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
    CurvaturePointRepresentation ()
    {
        nr_dimensions_ = 4;
    }

    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
    {
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

class IncrementalRegister : public QObject
{
    Q_OBJECT

public:
    IncrementalRegister(float leafsize);
    ~IncrementalRegister();

    void registerCloud(PointCloud::ConstPtr cloud);

    bool removeLatestCloud();

    void reset();

    PointCloud::Ptr getCurrentCloud();

protected:
    void registerPair(PointCloud::Ptr src, PointCloud::Ptr tgt, Eigen::Matrix4f &pair_transformation);
    void addCloudToFinalModel(PointCloud::ConstPtr cloud, Eigen::Matrix4f transformation);
    void addTransformation(Eigen::Matrix4f &transformation);
    void removeLatestTransformation();
    Eigen::Matrix4f getGlobalTransformation();

private:
    std::vector<PointCloud::Ptr> final_clouds_;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations_;
    QMutex current_cloud_mutex_, registered_clouds_mutex_;

    CloudFilter *cloud_filter_;

    double icp_corr_distance_;
    double ransac_outlier_threshold_;
    double uniform_leafsize_;

signals:
    void globalModelUpdated(PointCloud::Ptr cloud);

};

}
