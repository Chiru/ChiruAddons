#pragma once

#include <QObject>
#include <QTextStream>

#include "pcl/PolygonMesh.h"

namespace ObjectCapture
{

class ColladaExporter : public QObject
{
    Q_OBJECT

public:
    ColladaExporter();
    ~ColladaExporter();

    void Export(pcl::PolygonMesh::Ptr inputmesh, QString filename);

private:
    void writeAssets();
    void writeCameras();
    void writeLights();
    void writeImages();
    void writeMaterials();
    void writeEffects();

    void writeGeometry();

    void writeControllers();
    void writeVisualScenes();
    void writeScene();

    pcl::PolygonMesh::Ptr input_mesh_;
    QString filename_;
    QTextStream fs;

};

} //end of namespace ObjectCapture
