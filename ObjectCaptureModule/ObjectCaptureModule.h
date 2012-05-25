/**
 *  For conditions of distribution and use, see copyright notice in license.txt
 *
 *  @file   ObjectCaptureModule.h
 *  @brief  Chiru 3D object capture and reconstruction module.
 */

#pragma once

#include "IModule.h"
#include "CoreStringUtils.h"
#include "Math/float3.h"
#include "Math/Quat.h"
#include "Math/MathFwd.h"
#include "Entity.h"
#include "OgreModuleFwd.h"

#include "ObjectCaptureModuleDefines.h"
#include "pcl/PolygonMesh.h"

#include <QObject>
#include <QImage>
class QThread;

namespace ObjectCapture
{
class CloudProcessor;
class MeshReconstructor;
class MeshConverter;

/**
 *  Implementation of Chiru object capture.
 *
 *  Uses QXMPP library to provide presence, text and voice communications.
 *
 *  See usage example in <insert sample scene dir here>
 *
 *  Work in progress
 */
class ObjectCaptureModule : public IModule
{
    Q_OBJECT

    struct CloudPosition
    {
        Quat orientation;
        float3 position;
        float3 scale;
    };

public:
    /// Default constructor.
    ObjectCaptureModule();

    /// Destructor.
    ~ObjectCaptureModule();

    /// IModule override.
    void Load();

    /// IModule override.
    void Initialize();

    /// IModule override.
    void Uninitialize();

    /// IModule override.
    void Update(f64 frametime);


public slots:
    /// Starts the Kinect capturing interface
    void startCapturing();

    /// Stops the Kinect capturing interface without processing the clouds
    void stopCapturing();

    /// Saves current cloud
    void captureCloud();

    /// Stops capturing and processes the clouds to a mesh
    void finalizeCapturing();

    void setLiveCloudPosition(Quat orientation, float3 position, float3 scale);

    void setGlobalModelPosition(Quat orientation, float3 position, float3 scale);

    void setFinalMeshPosition(Quat orientation, float3 position, float3 scale);

signals:
    void previewFrameUpdated(const QImage &frame);
    void objectCaptured(unsigned int entityId);

private slots:
    void meshReconstructed();
    void addObjectToScene(EntityPtr entity, Ogre::ManualObject *mesh, Quat orientation, float3 position, float3 scale); // Refactor name?
    void visualizeLiveCloud(PointCloud::Ptr cloud);
    void visualizeGlobalModel(PointCloud::Ptr cloud);
    void visualizeFinalMesh(pcl::PolygonMesh::Ptr polygonMesh, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

private:
    CloudProcessor *cloud_processor_;
    MeshReconstructor *mesh_reconstructor_;
    MeshConverter *mesh_converter_;
    QThread *worker_thread_;

    struct CloudPosition live_cloud_position_;
    struct CloudPosition global_model_position_;
    struct CloudPosition final_mesh_position_;
};

} // end of namespace: ObjectCapture
