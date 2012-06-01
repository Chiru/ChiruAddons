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
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud = inputCloud;

    size_t vertexcount = input_cloud->points.size();
    size_t indicescount = 0;

    // estimation for the index count is triangle_size*3*2 and it is always larger than the real value
    // Calculate real value for indices
    for (size_t i = 0; i < inputMesh->polygons.size(); i++)
                indicescount += inputMesh->polygons[i].vertices.size();

    Ogre::ManualObject *ogreManual = createManualObject(vertexcount, indicescount, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    for (size_t i = 0; i < input_cloud->points.size(); i++)
    {
        Ogre::Real r = (Ogre::Real)input_cloud->points[i].r / (Ogre::Real)255;
        Ogre::Real g = (Ogre::Real)input_cloud->points[i].g / (Ogre::Real)255;
        Ogre::Real b = (Ogre::Real)input_cloud->points[i].b / (Ogre::Real)255;

        ogreManual->colour(r, g, b);
        ogreManual->position(input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z);
        ogreManual->normal(input_cloud->points[i].data_c[0], input_cloud->points[i].data_c[1], input_cloud->points[i].data_c[2]);
    }

    //Add indexing data to the manualobject
    for (size_t i = 0; i < inputMesh->polygons.size(); i++)
    {
        for (size_t j = 0; j < inputMesh->polygons[i].vertices.size(); j++)
        {
            int index = inputMesh->polygons[i].vertices[j];
            ogreManual->index(index);
        }
    }
    ogreManual->end();

    return ogreManual;
}

Ogre::ManualObject* MeshConverter::CreatePointMesh(PointCloud::Ptr inputCloud)
{
    size_t vertexcount = inputCloud->points.size();

    Ogre::ManualObject *ogreManual = createManualObject(vertexcount, vertexcount, Ogre::RenderOperation::OT_POINT_LIST);

    for (size_t i = 0; i < inputCloud->points.size(); i++)
    {
        Ogre::Real r = (Ogre::Real)inputCloud->points[i].r / (Ogre::Real)255;
        Ogre::Real g = (Ogre::Real)inputCloud->points[i].g / (Ogre::Real)255;
        Ogre::Real b = (Ogre::Real)inputCloud->points[i].b / (Ogre::Real)255;

        ogreManual->colour(r, g, b);
        ogreManual->position(inputCloud->points[i].x, inputCloud->points[i].y, inputCloud->points[i].z);
        ogreManual->index(i);
    }

    ogreManual->end();

    return ogreManual;
}

Ogre::ManualObject* MeshConverter::createManualObject(size_t vertexCount, size_t indicesCount, Ogre::RenderOperation::OperationType operationType)
{
    scene_ = framework_->Scene()->MainCameraScene();
    if (scene_)
        world_ = scene_->GetWorld<OgreWorld>();
    OgreWorldPtr world = world_.lock();
    Ogre::SceneManager* sceneMgr = world->OgreSceneManager();

    // ManualObject must be created within the same thread as the OgreRendering is running!
    Ogre::ManualObject *ogreManual = sceneMgr->createManualObject(world->GetUniqueObjectName("ImportedMesh"));

    //double startime = pcl::getTime();

    //LogInfo("MeshConverter: Begin of manual object creation");
    ogreManual->clear();
    ogreManual->estimateVertexCount(vertexCount);
    ogreManual->estimateIndexCount(indicesCount);
    ogreManual->begin("CapturedObject", operationType);
    ogreManual->setDynamic(false);

    //LogInfo("MeshConverter: Object created!");
    //double objectCreateTime = pcl::getTime() - startime;
    //LogInfo("MeshConverter: Time needed for CustomObject creation: " + ToString(objectCreateTime));

    return ogreManual;
}

} // end of namespace ObjectCapture
