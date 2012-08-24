// For conditions of distribution and use, see copyright notice in LICENSE

#include "StableHeaders.h"
#include "DebugOperatorNew.h"
#include "LoggingFunctions.h"
#include "CloudFilter.h"

#include "KinectCapture.h"

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcl_io_exception.h>

#include "MemoryLeakCheck.h"

namespace ObjectCapture
{

KinectCapture::KinectCapture() :
    cloud_filter_(new CloudFilter()),
    extract_object_(false)
{
    try
    {
        kinect_interface_ = new pcl::OpenNIGrabber();

        boost::function<void (const PointCloud::ConstPtr&)> f = boost::bind(&KinectCapture::kinect_callback_, this, _1);
        kinect_interface_->registerCallback(f);
    }
    catch(pcl::PCLException e)
    {
        LogInfo("ObjectCaptureModule: Caught PCL exception: " + QString::fromUtf8(e.what()));
        kinect_interface_ = NULL;
    }
}

KinectCapture::~KinectCapture()
{
    if(kinect_interface_) {
        kinect_interface_->stop();
        delete kinect_interface_;
    }
}

bool KinectCapture::getExtractObject()
{
    return extract_object_;
}

void KinectCapture::setExtractObject(bool value)
{
    extract_object_ = value;
}

void KinectCapture::startCapture()
{
    if(kinect_interface_) {
        kinect_interface_->start();
        LogDebug("ObjectCaptureModule: Starting Kinect interface..");
    }
}

void KinectCapture::stopCapture()
{
    if(kinect_interface_) {
        kinect_interface_->stop();
        LogDebug("ObjectCaptureModule: Stopping Kinect interface");
    }
}

bool KinectCapture::isRunning()
{
    if(!kinect_interface_)
        return false;
    else
        return kinect_interface_->isRunning();
}

PointCloud::Ptr KinectCapture::currentCloud()
{
    cloud_mutex_.lock();
    PointCloud::Ptr cloud;
    cloud = current_cloud_;
    cloud_mutex_.unlock();
    return cloud;
}

void KinectCapture::kinect_callback_(const PointCloud::ConstPtr &cloud)
{
    if(cloud_mutex_.tryLock())
    {
        if(extract_object_)
        {
            PointCloud::Ptr depth_filtered = cloud_filter_->filterDepth(cloud, 0.0f, 1.5f); // Move params to class members
            PointCloud::Ptr downsampled = cloud_filter_->filterDensity(depth_filtered, 0.01f); // --*--

            PointCloud::Ptr clustered = cloud_filter_->segmentCloud(downsampled, 0.08);
            if(clustered->points.size() <= 0) // Segmentation failed, skip this frame.
            {
                cloud_mutex_.unlock();
                return;
            }

            current_cloud_ = clustered;
        }
        else
        {
            pcl::copyPointCloud(*cloud, *current_cloud_);
        }
        cloud_mutex_.unlock();

        emit cloudUpdated(current_cloud_);
    }
}

}
