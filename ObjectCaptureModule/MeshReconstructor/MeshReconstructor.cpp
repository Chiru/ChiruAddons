// For conditions of distribution and use, see copyright notice in LICENSE

#include "StableHeaders.h"
#include "DebugOperatorNew.h"
#include "IModule.h"

#include "MeshReconstructor.h"

#include <pcl/surface/reconstruction.h>

#include <pcl/surface/mls_omp.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include "pcl/common/common_headers.h"

#include "pcl/features/normal_3d.h"
#include <pcl/point_traits.h>
#include <pcl/registration/transforms.h>

#include <boost/thread/thread.hpp>

#include <pcl/surface/gp3.h>
#include <pcl/common/time.h>

#include "LoggingFunctions.h"
#include "CoreStringUtils.h"
#include "MemoryLeakCheck.h"

namespace ObjectCapture
{

MeshReconstructor::MeshReconstructor() :
    point_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>()),
    normals_(new SurfaceNormals()),
    smoothed_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>()),
    cloud_with_normals_(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
    polygonMesh_(new pcl::PolygonMesh())
{

}

MeshReconstructor::~MeshReconstructor()
{
    point_cloud_.reset();
    normals_.reset();
    smoothed_cloud_.reset();
    cloud_with_normals_.reset();
    polygonMesh_.reset();
}

void MeshReconstructor::processCloud(PointCloud::Ptr cloud)
{
    if (cloud.get())
    {
        pcl::PointCloud<pcl::PointXYZRGB> tmp_cloud;

        // Convert input cloud PointXYZRGBA to PointXYZRGB cloud and rotate it 180 degrees by the x-axis
        tmp_cloud.resize(cloud->size());
        for (size_t i = 0; i < cloud->points.size(); i++)
        {
            tmp_cloud.points[i].x = cloud->points[i].x;
            tmp_cloud.points[i].y = cloud->points[i].y;
            tmp_cloud.points[i].z = cloud->points[i].z;
            tmp_cloud.points[i].r = cloud->points[i].r;
            tmp_cloud.points[i].g = cloud->points[i].g;
            tmp_cloud.points[i].b = cloud->points[i].b;
        }

        Eigen::Quaternionf RotQuat(0.0, 1.0, 0.0, 0.0);
        Eigen::Vector3f vect(0.0, 0.0, 0.0);

        pcl::transformPointCloud(tmp_cloud, *point_cloud_, vect, RotQuat);

        MovingLeastSquares();
        NormalEstimation();
        GreedyProjection_Mesher();
        //convertVtkToMesh();

        //cloud_with_normals_->resize(cloud_with_normals_->points.size());
        emit cloudProcessingFinished(polygonMesh_, cloud_with_normals_);
    }
    else
        LogError("ObjectCaptureModule: Couldn't get input cloud for ObjectReconstruction!");
}

void MeshReconstructor::convertVtkToMesh()
{
    // Debrecated
    if (!polygonMesh_)
        return;

    double starttime = pcl::getTime();
    char Command[100];
    std::stringstream filename;

    filename << "testmesh.vtk";

    LogInfo( "Saving file: " + filename.str());

    // Save polygon mesh to vtk file
    pcl::io::saveVTKFile(filename.str(), *polygonMesh_);

    sprintf(Command, "python OgreMeshXML.py -i testmesh.vtk -o testmesh.xml");
    if (system(Command))
        LogError("ObjectCapture: Error while converting vtk to xml!\n");

    sprintf(Command, "OgreXMLConverter testmesh.xml testmesh.mesh");
    if (system(Command))
        LogError("ObjectCapture: Error while converting xml to mesh!\n");

    sprintf(Command, "mv testmesh.mesh data/assets/");
    if (system(Command))
        LogError("ObjectCapture: Error while moving mesh to scenes folder!\n");

    LogInfo("Mesh conversion time: " + ToString(pcl::getTime() - starttime));
}

void MeshReconstructor::MovingLeastSquares()
{
    LogInfo("ObjectCapture: Smoothing object geometry..");
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>(false));

    // Init object (second point type is for the output)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;

    if (point_cloud_->points.size() > 0)
    {
        // Set parameters
        mls.setInputCloud (point_cloud_);
        mls.setPolynomialFit (true);
        mls.setPolynomialOrder(2);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.1); //0.03
        mls.setSqrGaussParam(0.009); //0.0009

        // Reconstruct
        mls.process (*smoothed_cloud_);

        //Restore color data missed during the smoothing process.
        for (size_t i = 0; i < point_cloud_->points.size(); i++)
        {
            smoothed_cloud_->points[i].r = point_cloud_->points[i].r;
            smoothed_cloud_->points[i].g = point_cloud_->points[i].g;
            smoothed_cloud_->points[i].b = point_cloud_->points[i].b;
        }
    }
}

void MeshReconstructor::NormalEstimation()
{

    LogInfo("ObjectCapture: Estimating normals..");
    float normal_search_radius = 0.1; //0.0025;

    //initialize normal estimation
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    if (smoothed_cloud_.get())
    {
        normal_estimation.setInputCloud (smoothed_cloud_);

        //initialize search tree
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
        //search_tree->setInputCloud (smoothed_cloud_);

        normal_estimation.setSearchMethod (search_tree);
        //normal_estimation.setKSearch (20); //if we want more accurate results, re-run with KSearch!
        normal_estimation.setRadiusSearch (normal_search_radius);

        normal_estimation.compute (*normals_);
    }
}

void MeshReconstructor::GreedyProjection_Mesher()
{
    LogInfo("ObjectCapture: Reconstructing object surface..");
    // beging of meshing

    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> mesher;

    if (smoothed_cloud_.get() && normals_.get())
    {
        pcl::concatenateFields(*smoothed_cloud_, *normals_, *cloud_with_normals_);
    }
    else
    {
        LogError("ObjectCaptureModule: Couldn't get input cloud or point normals for reconstruction!");
        return;
    }

    mesher.setInputCloud(cloud_with_normals_);

    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    mesher.setSearchMethod(tree2);

    mesher.setSearchRadius(0.5);//0.05
    mesher.setConsistentVertexOrdering(true); // Requires PCL-1.5 or higher!

    mesher.setMu(5.0); //2.5
    mesher.setMaximumNearestNeighbors(800);
    mesher.setMaximumSurfaceAngle(M_PI); // M_PI/4
    mesher.setMinimumAngle(M_PI/18); // 10 degrees
    mesher.setMaximumAngle(2*M_PI/3); // 120 degrees
    mesher.setNormalConsistency(false);

    mesher.reconstruct(*polygonMesh_);
}
} //end of namespace ObjectCapture

