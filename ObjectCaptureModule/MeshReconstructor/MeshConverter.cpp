#include "MeshConverter.h"
#include "ObjectCaptureModuleDefines.h"

#include "EC_OgreCustomObject.h"
#include "Ogre.h"
#include "OgreWorld.h"
#include "OgreManualObject.h"

#include "OgreRenderingModule.h"
#include "Framework.h"
#include "SceneAPI.h"
#include "Scene.h"
#include "EC_Placeable.h"
#include "pcl/common/time.h"

#include "LoggingFunctions.h"

namespace ObjectCapture
{

MeshConverter::MeshConverter(Framework *framework) :
    framework_(framework),
    input_cloud_(new pcl::PointCloud<pcl::PointXYZRGBNormal>)
{
}

MeshConverter::~MeshConverter()
{
}

Ogre::ManualObject* MeshConverter::CreateMesh(pcl::PolygonMesh::Ptr inputMesh, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud)
{
    polygon_mesh_ = inputMesh;
    input_cloud_ = inputCloud;

    return createManualObject(Ogre::RenderOperation::OT_TRIANGLE_LIST);
}

Ogre::ManualObject* MeshConverter::CreatePointMesh(PointCloud::Ptr inputCloud)
{
    input_cloud_->points.resize(inputCloud->size());
    LogInfo("MeshReconstructor::processCloud poinst in cloud:" + ToString(inputCloud->size()));
    for (size_t i = 0; i < input_cloud_->points.size(); i++)
    {
        input_cloud_->points[i].x = inputCloud->points[i].x;
        input_cloud_->points[i].y = inputCloud->points[i].y;
        input_cloud_->points[i].z = inputCloud->points[i].z;
        input_cloud_->points[i].r = inputCloud->points[i].r;
        input_cloud_->points[i].g = inputCloud->points[i].g;
        input_cloud_->points[i].b = inputCloud->points[i].b;

        input_cloud_->points[i].data_c[0] = 1;
        input_cloud_->points[i].data_c[1] = 1;
        input_cloud_->points[i].data_c[2] = 1;
    }

    return createManualObject(Ogre::RenderOperation::OT_POINT_LIST);
}

Ogre::ManualObject* MeshConverter::createManualObject(Ogre::RenderOperation::OperationType operationType)
{
    scene_ = framework_->Scene()->MainCameraScene();
    if (scene_)
        world_ = scene_->GetWorld<OgreWorld>();
    OgreWorldPtr world = world_.lock();
    Ogre::SceneManager* sceneMgr = world->OgreSceneManager();

    // ManualObject must be created within the same thread as the OgreRendering is running!
    Ogre::ManualObject *ogreManual = sceneMgr->createManualObject(world->GetUniqueObjectName("ImportedMesh"));

    double startime = pcl::getTime();

    //ogreManual->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
    //ogreManual->setUseIdentityProjection(true);
    //ogreManual->setUseIdentityView(true);
    //ogreManual->setQueryFlags(0);

    // estimation for the index count is triangle_size*3*2 and it is always larger than the real value
    size_t triangle_size = input_cloud_->points.size();
    size_t indices_count = 0;

    if (operationType == Ogre::RenderOperation::OT_TRIANGLE_LIST)
    {
        for (size_t i = 0; i < polygon_mesh_->polygons.size(); i++)
            indices_count += polygon_mesh_->polygons[i].vertices.size();
    }
    else
        indices_count = triangle_size;

    //LogInfo("MeshConverter: Vertex count: "+ ToString(triangle_size));
    //LogInfo("MeshConverter: Index count: "+ ToString(indices_count));

    //LogInfo("MeshConverter: Begin of manual object creation");
    ogreManual->clear();
    ogreManual->estimateVertexCount(triangle_size);
    ogreManual->estimateIndexCount(indices_count);
    ogreManual->begin("CapturedObject", operationType);
    ogreManual->setDynamic(false);

    //LogInfo("MeshConverter: Setting positions...");

    for (size_t i = 0; i < input_cloud_->points.size(); i++)
    {
        Ogre::Real r = (Ogre::Real)input_cloud_->points[i].r / (Ogre::Real)255;
        Ogre::Real g = (Ogre::Real)input_cloud_->points[i].g / (Ogre::Real)255;
        Ogre::Real b = (Ogre::Real)input_cloud_->points[i].b / (Ogre::Real)255;

        ogreManual->colour(r, g, b);
        ogreManual->position(input_cloud_->points[i].x, input_cloud_->points[i].y, input_cloud_->points[i].z);

        if (operationType == Ogre::RenderOperation::OT_POINT_LIST)
            ogreManual->index(i);
        else
            ogreManual->normal(input_cloud_->points[i].data_c[0], input_cloud_->points[i].data_c[1], input_cloud_->points[i].data_c[2]);
    }

    if (operationType == Ogre::RenderOperation::OT_TRIANGLE_LIST)
    {
        //LogInfo("MeshConverter: OperationType == OT_TRIANGLE_LIST");
        for (size_t i = 0; i < polygon_mesh_->polygons.size(); i++)
        {
            for (size_t j = 0; j < polygon_mesh_->polygons[i].vertices.size(); j++)
            {
                int index = polygon_mesh_->polygons[i].vertices[j];
                ogreManual->index(index);
            }
        }
    }

    ogreManual->end();

    //LogInfo("MeshConverter: Object created!");
    double objectCreateTime = pcl::getTime() - startime;
    LogInfo("MeshConverter: Time needed for CustomObject creation: " + ToString(objectCreateTime));

    //this->addMeshToScene(ogreManual);
    return ogreManual;
}



} // end of namespace ObjectCapture
