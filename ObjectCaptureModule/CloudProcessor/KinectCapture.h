// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "ObjectCaptureModuleDefines.h"

#include <QObject>
#include <QImage>
#include <QTimer>
#include <QMutex>

namespace pcl
{
    class Grabber;
}

namespace ObjectCapture
{
class CloudFilter;

class KinectCapture : public QObject
{
    Q_OBJECT

public:
    KinectCapture(float leafsize);
    ~KinectCapture();

    /// Returns wheter kinect interface is running
    bool isRunning();

    /// Returns wether object extraction is in use or not
    bool getExtractObject();

    /// Set object extraction on or off
    /// \param value
    void setExtractObject(bool value);

    /// Set planar filter on or off
    /// \param value
    /// \note has no effect if object extraction is turned off
    void setFilterPlanar(bool value);

    /// Returns wether planar filtering is on or off
    bool getFilterPlanar();

    void kinect_callback_ (const PointCloud::ConstPtr &cloud);

public slots:
    void startCapture();
    void stopCapture();
    PointCloud::Ptr currentCloud();

signals:
    void cloudUpdated(PointCloud::Ptr cloud);

private slots:

private:
    pcl::Grabber *kinect_interface_;
    PointCloud::Ptr current_cloud_;
    QMutex cloud_mutex_;
    CloudFilter *cloud_filter_;
    float uniform_leafsize_;

    bool extract_object_;
    bool filter_planar_;
};

}
