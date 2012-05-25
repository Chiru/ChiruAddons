#include "MeshConverter.h"
#include "ObjectCaptureModuleDefines.h"

#include "EC_OgreCustomObject.h"
#include "OgreSceneManager.h"
#include "OgreWorld.h"
#include "OgreManualObject.h"

#include "Framework.h"
#include "SceneAPI.h"
#include "Scene.h"
#include "EC_Placeable.h"
#include "pcl/common/time.h"

#include "LoggingFunctions.h"

namespace ObjectCapture
{

MeshConverter::MeshConverter(Framework *framework) :
    framework_(framework)
{
}

MeshConverter::~MeshConverter()
{
}

Ogre::ManualObject* MeshConverter::CreateMesh(pcl::PolygonMesh::Ptr inputMesh, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud)
{
    Ogre::ManualObject *manual_object = createManualObject(inputCloud, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    //Add indexing data to the manualobject
    for (size_t i = 0; i < inputMesh->polygons.size(); i++)
    {
        for (size_t j = 0; j < inputMesh->polygons[i].vertices.size(); j++)
        {
            int index = inputMesh->polygons[i].vertices[j];
            manual_object->index(index);
        }
    }
    manual_object->end();

    return manual_object;
}

Ogre::ManualObject* MeshConverter::CreatePointMesh(PointCloud::Ptr inputCloud)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    input_cloud->points.resize(inputCloud->size());
    //LogInfo("MeshReconstructor::processCloud poinst in cloud:" + ToString(inputCloud->size()));

    //Convert PointXYZRGBA cloud to PointXYZRGBNormal cloud
    for (size_t i = 0; i < input_cloud->points.size(); i++)
    {
        input_cloud->points[i].x = inputCloud->points[i].x;
        input_cloud->points[i].y = inputCloud->points[i].y;
        input_cloud->points[i].z = inputCloud->points[i].z;
        input_cloud->points[i].r = inputCloud->points[i].r;
        input_cloud->points[i].g = inputCloud->points[i].g;
        input_cloud->points[i].b = inputCloud->points[i].b;

        //Normals
        input_cloud->points[i].data_c[0] = 1;
        input_cloud->points[i].data_c[1] = 1;
        input_cloud->points[i].data_c[2] = 1;
    }

    Ogre::ManualObject *object = createManualObject(input_cloud, Ogre::RenderOperation::OT_POINT_LIST);
    object->end();

    return object;
}

Ogre::ManualObject* MeshConverter::createManualObject(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud, Ogre::RenderOperation::OperationType operationType)
{
    scene_ = framework_->Scene()->MainCameraScene();
    if (scene_)
        world_ = scene_->GetWorld<OgreWorld>();
    OgreWorldPtr world = world_.lock();
    Ogre::SceneManager* sceneMgr = world->OgreSceneManager();

    // ManualObject must be created within the same thread as the OgreRendering is running!
    Ogre::ManualObject *ogreManual = sceneMgr->createManualObject(world->GetUniqueObjectName("ImportedMesh"));

    //double startime = pcl::getTime();

    // estimation for the index count is triangle_size*3*2 and it is always larger than the real value
    size_t triangle_size = input_cloud->points.size();
    size_t indices_count = triangle_size*3*2;

    //LogInfo("MeshConverter: Vertex count: "+ ToString(triangle_size));
    //LogInfo("MeshConverter: Index count: "+ ToString(indices_count));

    //LogInfo("MeshConverter: Begin of manual object creation");
    ogreManual->clear();
    ogreManual->estimateVertexCount(triangle_size);
    ogreManual->estimateIndexCount(indices_count);
    ogreManual->begin("CapturedObject", operationType);
    ogreManual->setDynamic(false);

    //LogInfo("MeshConverter: Setting positions...");

    for (size_t i = 0; i < input_cloud->points.size(); i++)
    {
        Ogre::Real r = (Ogre::Real)input_cloud->points[i].r / (Ogre::Real)255;
        Ogre::Real g = (Ogre::Real)input_cloud->points[i].g / (Ogre::Real)255;
        Ogre::Real b = (Ogre::Real)input_cloud->points[i].b / (Ogre::Real)255;

        ogreManual->colour(r, g, b);
        ogreManual->position(input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z);

        if (operationType == Ogre::RenderOperation::OT_POINT_LIST)
            ogreManual->index(i);
        else
            ogreManual->normal(input_cloud->points[i].data_c[0], input_cloud->points[i].data_c[1], input_cloud->points[i].data_c[2]);
    }

    //LogInfo("MeshConverter: Object created!");
    //double objectCreateTime = pcl::getTime() - startime;
    //LogInfo("MeshConverter: Time needed for CustomObject creation: " + ToString(objectCreateTime));

    return ogreManual;
}

} // end of namespace ObjectCapture
