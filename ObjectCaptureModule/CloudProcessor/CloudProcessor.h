// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "ObjectCaptureModuleDefines.h"

#include <QList>
#include <QObject>
#include <QImage>

namespace ObjectCapture
{
class KinectCapture;
class CloudFilter;
class IncrementalRegister;

class CloudProcessor : public QObject
{
    Q_OBJECT

public:
    CloudProcessor();
    ~CloudProcessor();

public slots:
    /// Starts the capturing interface
    void startCapture();

    /// Stops the capturing interface
    void stopCapture();

    /// Captures a point cloud and passes it for registration
    void captureCloud();

    /// Removes latest cloud from the global model
    void rewindCloud();

    /// Stops the capturing interface and passes the cloud for surface reconstruction
    void finalizeCapturing();

    void setFilterPlanar(bool value);

    /// Returns boost shared pointer to final point cloud
    PointCloud::Ptr finalCloud() const;

signals:
    void liveFeedUpdated(const QImage &frame);
    void globalModelUpdated(PointCloud::Ptr cloud);
    void liveCloudUpdated(PointCloud::Ptr cloud);

private slots:
    void handleGlobalModelUpdated(PointCloud::Ptr cloud);
    void handleLiveCloudUpdated(PointCloud::Ptr cloud);

protected:
    void moveToOrigo(PointCloud::Ptr cloud);

private:
    KinectCapture *kinect_capture_;
    CloudFilter *cloud_filter_;
    QList<PointCloud::ConstPtr> captured_clouds_;
    PointCloud::Ptr final_cloud_;
    PointCloud::Ptr live_cloud_;
    IncrementalRegister *register_;
};

}
