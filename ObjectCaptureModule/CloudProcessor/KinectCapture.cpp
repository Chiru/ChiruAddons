// For conditions of distribution and use, see copyright notice in LICENSE

#include "StableHeaders.h"
#include "DebugOperatorNew.h"
#include "LoggingFunctions.h"
#include "CloudFilter.h"

#include "KinectCapture.h"

#include <pcl/io/openni_grabber.h>
#include <QRgb>

#include "MemoryLeakCheck.h"

namespace ObjectCapture
{

KinectCapture::KinectCapture() :
    cloud_filter_(new CloudFilter()),
    current_cluster_(PointCloud::Ptr(new PointCloud)),
    extract_object_(false),
    rgb_update_frequency_(15)
{
    kinect_interface_ = new pcl::OpenNIGrabber();
    rgb_frame_ = QImage(640, 480, QImage::Format_ARGB32);

    boost::function<void (const PointCloud::ConstPtr&)> f = boost::bind(&KinectCapture::kinect_callback_, this, _1);
    kinect_interface_->registerCallback(f);

    //bool check;
    //check = connect(&rgb_update_timer_, SIGNAL(timeout()), this, SLOT(updateRGBImage()));

    rgb_update_timer_.start(1000/rgb_update_frequency_);
}

KinectCapture::~KinectCapture()
{
    kinect_interface_->stop();
    delete kinect_interface_;
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

PointCloud::ConstPtr KinectCapture::currentCloud()
{
    cloud_mutex_.lock();
    PointCloud::ConstPtr cloud;
    if(extract_object_)
        cloud = current_cluster_;
    else
        cloud = current_cloud_;
    cloud_mutex_.unlock();
    return cloud;
}

void KinectCapture::updateRGBImage()
{
    static bool first_time = true; // for debugging

    /// \todo need to create new image for each frame?
    /// \todo figure out a way avoid innecessary copying of rgb values.
    if(!kinect_interface_->isRunning())
        return;

    if(cloud_mutex_.tryLock())
    {
        if(!current_cloud_.get())
        {
            cloud_mutex_.unlock();
            return;
        }


        PointCloud::ConstPtr cloud = current_cloud_;
        PointCloud::Ptr cluster;
        if(extract_object_)
            cluster = current_cluster_;

        cloud_mutex_.unlock();

        cloud->points.size(); // Just here to ensure compiler doesn't optimize cloud out

        emit currentClusterUpdated(current_cluster_);

//        QRgb value;
//        int cluster_index = 0;
//        for(int u = 0; u < 480; u++)
//        {
//            for(int v = 0; v < 640; v++)
//            {
//                int cloud_index = (u*640)+v;
//                if(isnan(cloud->points[cloud_index].x))
//                    //value = qRgb(255, 63, 139);
//                    value = qRgb(0, 0, 0);
//                else
//                    value = qRgb(cloud->points[cloud_index].r, cloud->points[cloud_index].g, cloud->points[cloud_index].b);

//                // Experimental
//                /*
//                float px = cloud->points[cloud_index].x;
//                float py = cloud->points[cloud_index].y;
//                float pz = cloud->points[cloud_index].z;

//                if(extract_object_ && !first_time)
//                {
//                    for(int i = 0; i < cluster->points.size(); ++i)
//                    {
//                        float qx = cluster->points[i].x;
//                        float qy = cluster->points[i].y;
//                        float qz = cluster->points[i].z;
//                        // Calculate Euclidean distance
//                        float distance = sqrt(pow(px-qx, 2.0f) + pow(py-qy, 2.0f) + pow(pz-qz, 2.0f));
//                        if(distance <= 0.01) // change to use the same distance as uniform sampling
//                        {
//                            unsigned int red = ((value >> 16) & 0xff) + 0x64;
//                            red = (red <= 0xff) ? red : 0xff;
//                            value = (value | ((red) << 16));
//                            break;
//                        }
//                    }
//                }*/

//                rgb_frame_.setPixel(v, u, value);
//                first_time = false;
//            }
//        }

//        emit RGBUpdated(rgb_frame_);
    }
}

void KinectCapture::kinect_callback_(const PointCloud::ConstPtr &cloud)
{
    if(cloud_mutex_.tryLock()) {
        current_cloud_.reset();
        current_cloud_ = cloud;
        cloud_mutex_.unlock();

        if(extract_object_)
        {
            cloud_mutex_.lock();

            PointCloud::Ptr depth_filtered = cloud_filter_->filterDepth(current_cloud_, 0.0f, 1.5f); // Move params to class members
            PointCloud::Ptr downsampled = cloud_filter_->filterDensity(depth_filtered, 0.01f); // --*--

            current_cluster_ = cloud_filter_->segmentCloud(downsampled, 0.08);
            cloud_mutex_.unlock();
            updateRGBImage();
            emit cloudUpdated(current_cluster_);
            return;
        }
        updateRGBImage();
        emit cloudUpdated(current_cloud_);
    }
}

}
