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
#include "OgreMaterial.h"
#include "IAssetTransfer.h"

#include "ObjectCaptureModuleDefines.h"
#include "pcl/PolygonMesh.h"
#include "AssetFwd.h"

#include <QObject>
#include <QImage>
class QThread;

namespace ObjectCapture
{
class CloudProcessor;
class MeshReconstructor;
class MeshConverter;
class ColladaExporter;

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

    /// Removes the latest captured cloud
    void rewindCloud();

    /// Stops capturing and processes the clouds to a mesh
    void finalizeCapturing();

    /// Converts and saves the latest captured object in collada format
    /// \param filename filename for the collada file
    void exportCollada(QString filename);

    /// Sets the inworld transformation of the live point cloud
    void setLiveCloudPosition(Quat orientation, float3 position, float3 scale);

    /// Sets the inworld transformation of the global model
    void setGlobalModelPosition(Quat orientation, float3 position, float3 scale);

    /// Sets the inworld transformation of the final captured object
    void setFinalMeshPosition(Quat orientation, float3 position, float3 scale);

    /// Update point size for point meshes created from Kinect live feed and captured global model
    void updatePointSize();

    /// Sets URL used as a remote storage for collada files and add it to AssetStorage
    void setRemoteStorageURL(QString remoteStorageURL);

    void setFilterPlanar(bool value);

signals:
    void previewFrameUpdated(const QImage &frame);
    void objectCaptured(unsigned int entityId);
    void colladaExportFinalized(QString relativeFilePath);
    void assetUploaded(QString assetRef);

private slots:
    void meshReconstructed();
    void addObjectToScene(EntityPtr entity, Ogre::ManualObject *mesh, Quat orientation, float3 position, float3 scale); // Refactor name?
    void visualizeLiveCloud(PointCloud::Ptr cloud);
    void visualizeGlobalModel(PointCloud::Ptr cloud);
    void visualizeFinalMesh(pcl::PolygonMesh::Ptr polygonMesh, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    void uploadAsset(QString localAssetPath);
    void assetUploadComplete(QString assetRef);
    void assetUploadFailed(IAssetUploadTransfer *transfer);
    Ogre::MaterialPtr createMaterial(QString materialName);
    void storageAdded(AssetStoragePtr storage);

private:
    CloudProcessor *cloud_processor_;
    MeshReconstructor *mesh_reconstructor_;
    MeshConverter *mesh_converter_;
    ColladaExporter *collada_exporter_;
    QThread *worker_thread_;

    pcl::PolygonMesh::Ptr final_polygon_mesh_;

    EntityPtr live_cloud_entity;
    EntityPtr global_model_entity;
    EntityPtr final_mesh_entity;

    struct CloudPosition live_cloud_position_;
    struct CloudPosition global_model_position_;
    struct CloudPosition final_mesh_position_;

    QStringList assetUploads_;
    AssetStoragePtr remoteColladaStorage_;
};

} // end of namespace: ObjectCapture
