#pragma once

#include <QObject>

#include "ObjectCaptureModuleDefines.h"

#include "pcl/PolygonMesh.h"
#include "OgreModuleFwd.h"
#include "OgreRenderOperation.h"

class Framework;
class Scene;

namespace ObjectCapture
{

class MeshConverter : public QObject
{
    Q_OBJECT

public:
    MeshConverter(Framework *framework);
    ~MeshConverter();

public slots:
    Ogre::ManualObject* CreateMesh(pcl::PolygonMesh::Ptr inputMesh, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud);
    Ogre::ManualObject* CreatePointMesh(PointCloud::Ptr inputCloud, std::string materialName);

private slots:
    /// @return Pointer to unclosed Ogre ManualObject
    Ogre::ManualObject* createManualObject(size_t vertexCount, size_t indicesCount, std::string materialName, Ogre::RenderOperation::OperationType operationType);

private:
    Framework *framework_;
    OgreWorldWeakPtr world_;
    Scene *scene_;
};

} // end of namespace ObjectCapture
