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

    PointCloud::Ptr finalCloud() const;

public slots:
    void startCapture();
    void stopCapture();
    void captureCloud();
    void rewindCloud();
    void finalizeCapturing();

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
