// For conditions of distribution and use, see copyright notice in license.txt

#include <QThread>

#include "StableHeaders.h"
#include "DebugOperatorNew.h"

#include "ObjectCaptureModule.h"
#include <pcl/io/pcd_io.h>
#include "CloudProcessor/CloudProcessor.h"
#include "MeshReconstructor/MeshReconstructor.h"
#include "MeshReconstructor/MeshConverter.h"

#include "Framework.h"
#include "SceneAPI.h"
#include "Scene.h"
#include "IComponent.h"
#include "AssetReference.h"

#include "Entity.h"
#include "EC_Mesh.h"
#include "EC_Placeable.h"

#include "LoggingFunctions.h"

#include "MemoryLeakCheck.h"

namespace ObjectCapture
{
ObjectCaptureModule::ObjectCaptureModule() :
    IModule("ObjectCapture"),
    cloud_processor_(new CloudProcessor()),
    mesh_reconstructor_(new MeshReconstructor()),
    mesh_converter_(new MeshConverter(this)),
    worker_thread_(new QThread)
{
    qRegisterMetaType<PointCloud>("PointCloud");
    qRegisterMetaType<PointCloud::Ptr>("PointCloud::Ptr");
    qRegisterMetaType<pcl::PolygonMesh>("pcl::PolygonMesh");
    qRegisterMetaType<pcl::PolygonMesh::Ptr>("pcl::PolygonMesh::Ptr");
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBNormal> >("pcl::PointCloud<pcl::PointXYZRGBNormal>");
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>("pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr");

    mesh_reconstructor_->moveToThread(worker_thread_);
    cloud_processor_->moveToThread(worker_thread_);
    worker_thread_->start();

    bool check;
    check = connect(cloud_processor_, SIGNAL(RGBUpdated(QImage)), this, SIGNAL(previewFrameUpdated(QImage)), Qt::QueuedConnection);
    Q_ASSERT(check);

    check = connect(cloud_processor_, SIGNAL(registrationFinished(PointCloud::Ptr)), mesh_reconstructor_, SLOT(processCloud(PointCloud::Ptr)), Qt::QueuedConnection);
    Q_ASSERT(check);

    check = connect(mesh_reconstructor_, SIGNAL(cloudProcessingFinished(pcl::PolygonMesh::Ptr, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr)),
                    mesh_converter_, SLOT(Create(pcl::PolygonMesh::Ptr, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr)), Qt::QueuedConnection);
    //check = connect(mesh_reconstructor_, SIGNAL(cloudProcessingFinished(pcl::PolygonMesh::Ptr, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr)), this, SLOT(meshReconstructed()), Qt::QueuedConnection);
    Q_ASSERT(check);
}

ObjectCaptureModule::~ObjectCaptureModule()
{
    worker_thread_->quit();
    SAFE_DELETE(cloud_processor_);
    SAFE_DELETE(mesh_reconstructor_);
    SAFE_DELETE(mesh_converter_);
    SAFE_DELETE(worker_thread_);
}

void ObjectCaptureModule::Load()
{
    framework_->RegisterDynamicObject("ObjectCapture", this);
}

void ObjectCaptureModule::Initialize()
{
}

void ObjectCaptureModule::Uninitialize()
{
}

void ObjectCaptureModule::Update(f64 frametime)
{
}

void ObjectCaptureModule::startCapturing()
{
    cloud_processor_->startCapture();
}

void ObjectCaptureModule::stopCapturing()
{
    cloud_processor_->stopCapture();
}

void ObjectCaptureModule::captureCloud()
{
    cloud_processor_->captureCloud();
}

void ObjectCaptureModule::finalizeCapturing()
{
    cloud_processor_->finalizeCapturing();
}

void ObjectCaptureModule::meshReconstructed()
{
    mesh_reconstructor_->convertVtkToMesh();

    Scene *scene = framework_->Scene()->MainCameraScene();

    EntityPtr entity_ = scene->CreateEntity(scene->NextFreeIdLocal(), QStringList(), AttributeChange::LocalOnly, false);
    if (!entity_)
        LogError("ObjectCapture: Error while creating entity for mesh");
    else
    {
        Entity *meshEntity = entity_.get();
        IComponent *iComponent = meshEntity->GetOrCreateComponent("EC_Mesh", AttributeChange::LocalOnly, false).get();
        if (iComponent)
        {
            EC_Mesh *mesh = dynamic_cast<EC_Mesh*>(iComponent);
            AssetReferenceList materials;
            materials.Append(AssetReference("local://CapturedObject.material"));
            mesh->SetMeshRef("local://testmesh.mesh");
            mesh->setmeshMaterial(materials);
            mesh->SetAdjustOrientation(Quat(1.0, 0.0, 0.0, 0.0));
        }

        iComponent = meshEntity->GetOrCreateComponent("EC_Placeable", AttributeChange::LocalOnly, false).get();
        if (iComponent)
        {
            EC_Placeable *placeable = dynamic_cast<EC_Placeable*>(iComponent);
            placeable->setvisible(true);
            placeable->SetScale(float3(7.0, 7.0, 7.0));
        }

        scene->EmitEntityCreated(meshEntity, AttributeChange::LocalOnly);
        emit objectCaptured(meshEntity->Id());
    }
}

}// end of namespace: ObjectCapture

extern "C"
{
    DLLEXPORT void TundraPluginMain(Framework *fw)
    {
        Framework::SetInstance(fw); // Inside this DLL, remember the pointer to the global framework object.
        IModule *module = new ObjectCapture::ObjectCaptureModule();
        fw->RegisterModule(module);
    }
}
