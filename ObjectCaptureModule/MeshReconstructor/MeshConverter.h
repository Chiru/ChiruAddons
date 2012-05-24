#pragma once

#include <QObject>

#include "ObjectCaptureModuleDefines.h"

#include "pcl/PolygonMesh.h"
#include "OgreModuleFwd.h"
#include "OgreRenderOperation.h"
#include "Entity.h"

class Framework;
class Scene;

namespace ObjectCapture
{

class MeshConverter : public QObject
{
    Q_OBJECT

public:
    MeshConverter(Framework *framework_);
    ~MeshConverter();

public slots:
    void CreateMesh(pcl::PolygonMesh::Ptr inputMesh, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud);
    void CreatePointMesh(PointCloud::Ptr inputCloud);

private slots:
    void createManualObject(Ogre::RenderOperation::OperationType operationType);
    void addMeshToScene(Ogre::ManualObject *mesh);

private:
    pcl::PolygonMesh::Ptr polygon_mesh_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud_;

    Framework *framework_;
    OgreWorldWeakPtr world_;
    Scene *scene_;

    Ogre::ManualObject *ogreManual_;
    EntityPtr entity_;

};

} // end of namespace ObjectCapture
