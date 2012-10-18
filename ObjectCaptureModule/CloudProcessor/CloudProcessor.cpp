// For conditions of distribution and use, see copyright notice in LICENSE

#include "StableHeaders.h"
#include "DebugOperatorNew.h"
#include "LoggingFunctions.h"
#include "Math/float3.h"

#include "CloudProcessor.h"
#include "KinectCapture.h"
#include "CloudFilter.h"
#include "Registering.h"

#include "pcl/filters/passthrough.h"

#include "MemoryLeakCheck.h"

namespace ObjectCapture
{

CloudProcessor::CloudProcessor() :
    kinect_capture_(new KinectCapture(0.007f)),
    cloud_filter_(new CloudFilter()),
    register_(new IncrementalRegister(0.005f))
{
    kinect_capture_->setExtractObject(true);

    bool check;
    check = connect(kinect_capture_, SIGNAL(cloudUpdated(PointCloud::Ptr)), this, SLOT(handleLiveCloudUpdated(PointCloud::Ptr)));
    Q_ASSERT(check);

    check = connect(register_, SIGNAL(globalModelUpdated(PointCloud::Ptr)), this, SLOT(handleGlobalModelUpdated(PointCloud::Ptr)));
    Q_ASSERT(check);
}

CloudProcessor::~CloudProcessor()
{
    SAFE_DELETE(kinect_capture_);
    SAFE_DELETE(cloud_filter_);
    SAFE_DELETE(register_);
}

void CloudProcessor::startCapture()
{
    if(!kinect_capture_->isRunning())
        kinect_capture_->startCapture();
}

void CloudProcessor::stopCapture()
{
    if(kinect_capture_->isRunning())
        kinect_capture_->stopCapture();
}

void CloudProcessor::restartCapturing()
{
    if(!kinect_capture_->isRunning())
        kinect_capture_->startCapture();

    register_->reset();
}

void CloudProcessor::captureCloud()
{
    if(kinect_capture_->isRunning() && kinect_capture_->currentCloud().get())
        register_->registerCloud(kinect_capture_->currentCloud());
}

void CloudProcessor::rewindCloud()
{
    register_->removeLatestCloud();
}

void CloudProcessor::handleGlobalModelUpdated(PointCloud::Ptr cloud)
{
    if(cloud.get())
    {
        final_cloud_.reset(new PointCloud);
        pcl::copyPointCloud(*cloud, *final_cloud_);
        moveToOrigo(final_cloud_);
        emit globalModelUpdated(final_cloud_);
    }
}

void CloudProcessor::handleLiveCloudUpdated(PointCloud::Ptr cloud)
{
    if(cloud.get())
    {
        live_cloud_.reset(new PointCloud);
        pcl::copyPointCloud(*cloud, *live_cloud_);
        moveToOrigo(live_cloud_);
        emit liveCloudUpdated(live_cloud_);
    }
}

/// \todo unnecessary, refactor.
void CloudProcessor::finalizeCapturing()
{
    kinect_capture_->stopCapture();
}

void CloudProcessor::setFilterPlanar(bool value)
{
    if(kinect_capture_)
        kinect_capture_->setFilterPlanar(value);
}

void CloudProcessor::moveToOrigo(PointCloud::Ptr cloud)
{
    if(!cloud.get())
        return;

    float3 distance(0, 0, 0);
    for(unsigned i = 0; i < cloud->points.size(); ++i)
    {
        distance.x += cloud->points[i].x;
        distance.y += cloud->points[i].y;
        distance.z += cloud->points[i].z;
    }
    distance /= cloud->points.size();

    for(unsigned i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x -= distance.x;
        cloud->points[i].y -= distance.y;
        cloud->points[i].z -= distance.z;
    }
}

PointCloud::Ptr CloudProcessor::finalCloud() const
{
    return final_cloud_;
}

}
