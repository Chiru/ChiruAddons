// For conditions of distribution and use, see copyright notice in license.txt

#include <QThread>

#include "StableHeaders.h"
#include "DebugOperatorNew.h"

#include "ObjectCaptureModule.h"
#include <pcl/io/pcd_io.h>
#include "CloudProcessor/CloudProcessor.h"
#include "MeshReconstructor/MeshReconstructor.h"
#include "MeshReconstructor/MeshConverter.h"
#include "MeshReconstructor/ColladaExporter.h"

#include "Framework.h"
#include "SceneAPI.h"
#include "Scene.h"
#include "IComponent.h"
#include "AssetReference.h"
#include "EC_OgreCustomObject.h"

#include "OgreManualObject.h"
#include "OgreMaterialManager.h"
#include "OgreMaterial.h"
#include "OgreRenderingModule.h"
#include "OgreTechnique.h"
#include "Renderer.h"

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
    collada_exporter_(new ColladaExporter()),
    mesh_converter_(0),
    worker_thread_(new QThread)
{
    qRegisterMetaType<PointCloud>("PointCloud");
    qRegisterMetaType<PointCloud::Ptr>("PointCloud::Ptr");
    qRegisterMetaType<pcl::PolygonMesh>("pcl::PolygonMesh");
    qRegisterMetaType<pcl::PolygonMesh::Ptr>("pcl::PolygonMesh::Ptr");
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBNormal> >("pcl::PointCloud<pcl::PointXYZRGBNormal>");
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>("pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr");
}

ObjectCaptureModule::~ObjectCaptureModule()
{
    worker_thread_->quit();
    SAFE_DELETE(cloud_processor_);
    SAFE_DELETE(mesh_reconstructor_);
    SAFE_DELETE(collada_exporter_);
    SAFE_DELETE(mesh_converter_);
    SAFE_DELETE(worker_thread_);
}

void ObjectCaptureModule::Load()
{
    framework_->RegisterDynamicObject("ObjectCapture", this);
}

void ObjectCaptureModule::Initialize()
{
    mesh_converter_ = new MeshConverter(framework_);

    mesh_reconstructor_->moveToThread(worker_thread_);
    cloud_processor_->moveToThread(worker_thread_);
    worker_thread_->start();

    bool check;

    check = connect(cloud_processor_, SIGNAL(liveCloudUpdated(PointCloud::Ptr)), this, SLOT(visualizeLiveCloud(PointCloud::Ptr)), Qt::QueuedConnection);
    Q_ASSERT(check);

    check = connect(cloud_processor_, SIGNAL(globalModelUpdated(PointCloud::Ptr)), this, SLOT(visualizeGlobalModel(PointCloud::Ptr)), Qt::QueuedConnection);
    Q_ASSERT(check);

    check = connect(mesh_reconstructor_, SIGNAL(cloudProcessingFinished(pcl::PolygonMesh::Ptr, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr)),
                    this, SLOT(visualizeFinalMesh(pcl::PolygonMesh::Ptr,pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr)), Qt::QueuedConnection);

    Q_ASSERT(check);

    // Uggly
    live_cloud_position_.orientation = Quat(1,0,0,0);
    live_cloud_position_.position = float3(0,0,0);
    live_cloud_position_.scale = float3(0,0,0);

    global_model_position_.orientation = Quat(1,0,0,0);
    global_model_position_.position = float3(0,0,0);
    global_model_position_.scale = float3(0,0,0);

    final_mesh_position_.orientation = Quat(1,0,0,0);
    final_mesh_position_.position = float3(0,0,0);
    final_mesh_position_.scale = float3(0,0,0);
}

void ObjectCaptureModule::Uninitialize()
{
}

void ObjectCaptureModule::Update(f64 frametime)
{
}

void ObjectCaptureModule::startCapturing()
{
    QMetaObject::invokeMethod(cloud_processor_, "startCapture", Qt::QueuedConnection);
}

void ObjectCaptureModule::stopCapturing()
{
    QMetaObject::invokeMethod(cloud_processor_, "stopCapture", Qt::QueuedConnection);
}

void ObjectCaptureModule::captureCloud()
{
    QMetaObject::invokeMethod(cloud_processor_, "captureCloud", Qt::QueuedConnection);
}

void ObjectCaptureModule::rewindCloud()
{
    QMetaObject::invokeMethod(cloud_processor_, "rewindCloud", Qt::QueuedConnection);
}

void ObjectCaptureModule::finalizeCapturing()
{
    QMetaObject::invokeMethod(cloud_processor_, "finalizeCapturing", Qt::QueuedConnection);
    mesh_reconstructor_->processCloud(cloud_processor_->finalCloud());
}

void ObjectCaptureModule::setLiveCloudPosition(Quat orientation, float3 position, float3 scale)
{
    live_cloud_position_.orientation = orientation;
    live_cloud_position_.position = position;
    live_cloud_position_.scale = scale;
}

void ObjectCaptureModule::setGlobalModelPosition(Quat orientation, float3 position, float3 scale)
{
    global_model_position_.orientation = orientation;
    global_model_position_.position = position;
    global_model_position_.scale = scale;
}

void ObjectCaptureModule::setFinalMeshPosition(Quat orientation, float3 position, float3 scale)
{
    final_mesh_position_.orientation = orientation;
    final_mesh_position_.position = position;
    final_mesh_position_.scale = scale;
}

Ogre::MaterialPtr ObjectCaptureModule::createMaterial(QString materialName)
{
    // add the true as the last parameter to make it a manual material
    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(materialName.toStdString(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setReceiveShadows(false);

    Ogre::Pass *pass = material->getTechnique(0)->getPass(0);

    pass->setLightingEnabled( false );
    pass->setEmissive(0.1, 0.1, 0.1);

    pass->setCullingMode(Ogre::CULL_NONE);
    pass->setVertexColourTracking(Ogre::TVC_AMBIENT);
    pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    pass->setVertexColourTracking(Ogre::TVC_SPECULAR);

    material->load();

    return material;
}

void ObjectCaptureModule::visualizeLiveCloud(PointCloud::Ptr cloud)
{
    //static EntityPtr entity;
    static Ogre::ManualObject *previous_mesh = NULL;
    static Ogre::MaterialPtr material;

    if (!material.get())
        material = createMaterial("LiveCloud");

    Scene *scene = framework_->Scene()->MainCameraScene();

    if(!live_cloud_entity.get())
        live_cloud_entity = scene->CreateEntity(scene->NextFreeIdLocal(), QStringList(), AttributeChange::LocalOnly, false);

    Ogre::ManualObject *mesh = mesh_converter_->CreatePointMesh(cloud, material->getName());

    addObjectToScene(live_cloud_entity, mesh, live_cloud_position_.orientation, live_cloud_position_.position, live_cloud_position_.scale);

    delete previous_mesh; // Might crash depenting on EC_OgreCustomObject implementation
    previous_mesh = mesh;
}

void ObjectCaptureModule::visualizeGlobalModel(PointCloud::Ptr cloud)
{
    //static EntityPtr entity;
    static Ogre::ManualObject *previous_mesh = NULL;
    static Ogre::MaterialPtr material;

    // add the true as the last parameter to make it a manual material
    if (!material.get())
        material = createMaterial("GlobalModel");

    Scene *scene = framework_->Scene()->MainCameraScene();

    if(!global_model_entity.get())
        global_model_entity = scene->CreateEntity(scene->NextFreeIdLocal(), QStringList(), AttributeChange::LocalOnly, false);

    Ogre::ManualObject *mesh = mesh_converter_->CreatePointMesh(cloud, material->getName());

    addObjectToScene(global_model_entity, mesh, global_model_position_.orientation, global_model_position_.position, global_model_position_.scale);

    delete previous_mesh; // Might crash depenting on EC_OgreCustomObject implementation
    previous_mesh = mesh;
}

void ObjectCaptureModule::visualizeFinalMesh(pcl::PolygonMesh::Ptr polygonMesh, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    // Special case which requires script interaction.
    // Application needs to be aware of the final entity id, and in control of it's placement etc.

    final_polygon_mesh_ = polygonMesh;
    Scene *scene = framework_->Scene()->MainCameraScene();


    if(!final_mesh_entity.get())
        final_mesh_entity = scene->CreateEntity(scene->NextFreeIdLocal(), QStringList(), AttributeChange::LocalOnly, false);

    Ogre::ManualObject *mesh = mesh_converter_->CreateMesh(polygonMesh, cloud);
    addObjectToScene(final_mesh_entity, mesh, final_mesh_position_.orientation, final_mesh_position_.position, final_mesh_position_.scale);

    emit objectCaptured(final_mesh_entity->Id());
}

void ObjectCaptureModule::exportCollada(QString filename)
{
    if (final_polygon_mesh_.get())
    {
        collada_exporter_->Export(final_polygon_mesh_, filename);
        emit colladaExportFinalized(filename);
    }
    else
        LogError("Couldn't export polygon mesh! Mesh was not available.");
}

void ObjectCaptureModule::updatePointSize()
{
    OgreRenderer::RendererPtr renderer = framework_->GetModule<OgreRenderer::OgreRenderingModule>()->GetRenderer();
    Entity *parentEntity = renderer->MainCamera();
    EC_Placeable *camera_placeable = parentEntity->GetComponent<EC_Placeable>().get();

    float3 location = camera_placeable->WorldPosition();

    if (global_model_entity.get())
    {
        float global_model_distance = location.Distance(global_model_position_.position);
        float point_size = 80.0 / global_model_distance;

        Ogre::MaterialPtr m_pMtrl = Ogre::MaterialManager::getSingleton().load("GlobalModel", "General");
        m_pMtrl->getTechnique(0)->getPass(0)->setPointSize(point_size);
    }

    if (live_cloud_entity.get())
    {
        float live_cloud_distance = location.Distance(live_cloud_position_.position);
        float point_size = 80.0 / live_cloud_distance;

        Ogre::MaterialPtr m_pMtrl = Ogre::MaterialManager::getSingleton().load("LiveCloud", "General");
        m_pMtrl->getTechnique(0)->getPass(0)->setPointSize(point_size);
    }


}

void ObjectCaptureModule::addObjectToScene(EntityPtr entity, Ogre::ManualObject *mesh, Quat orientation, float3 position, float3 scale)
{
    //LogInfo("MeshConverter: create EC_OgreCustomObject");

    if (!entity.get())
        return;
        //entity_ = scene_->CreateEntity(scene_->NextFreeIdLocal(), QStringList(), AttributeChange::LocalOnly, false);

    ComponentPtr componentPtr = entity->GetOrCreateComponent("EC_OgreCustomObject", AttributeChange::LocalOnly, false);
    EC_OgreCustomObject *customObject = dynamic_cast<EC_OgreCustomObject*>(componentPtr.get());

    customObject->SetName("ImportedMesh");
    customObject->CommitChanges(mesh);

    // Get EC_Placeable for custom object
    componentPtr = entity->GetOrCreateComponent("EC_Placeable", AttributeChange::LocalOnly, false);
    EC_Placeable *placeable = dynamic_cast<EC_Placeable*>(componentPtr.get());
    placeable->SetTransform(orientation, position, scale);

    customObject->SetPlaceable(componentPtr);
}

void ObjectCaptureModule::meshReconstructed()
{
    //Deprecated!
    /// \todo Remove this slot

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
