#include "ColladaExporter.h"
#include <pcl/point_types.h>
#include <QFile>

namespace ObjectCapture
{

ColladaExporter::ColladaExporter()
{
}
ColladaExporter::~ColladaExporter()
{

}

void ColladaExporter::Export(pcl::PolygonMesh::Ptr inputmesh, QString filename)
{
    input_mesh_ = inputmesh;
    QFile file(filename);
    file.open(QIODevice::ReadWrite | QIODevice::Text);

    fs.setDevice(&file);
    fs << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    fs << "<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">\n"; //xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://www.collada.org/2005/11/COLLADASchema http://www.khronos.org/files/collada_schema_1_4\"
    // fs << "<COLLADA xmlns=\"http://www.collada.org/2008/03/COLLADASchema\" version=\"1.5.0\">";

    writeAssets();
    writeCameras();
    writeLights();
    writeImages();
    writeEffects();
    writeMaterials();
    writeGeometry();

    writeVisualScenes();
    writeScene();

    fs << "</COLLADA>\n";
    file.flush();
    file.close();

}

void ColladaExporter::writeAssets()
{
    /// \todo fix hard coded fields!
    fs << "   <asset>\n";
    fs << "       <contributor>\n";
    fs << "           <author>ObjectCapture User</author>\n";
    fs << "           <authoring_tool>ObjectCapture module</authoring_tool>\n";
    //fs << "           <comments> </comments>\n";
    fs << "           <copyright>Copyright</copyright>\n";
    fs << "       </contributor>\n";
    fs << "       <created>2012-07-15T11:41:05Z</created>\n";
    fs << "       <modified>2012-07-16T11:41:05Z</modified>\n";
    fs << "       <unit meter=\"1\" name=\"meter\"/>\n";
    fs << "       <up_axis>Y_UP</up_axis>\n";
    fs << "   </asset>\n";
}

void ColladaExporter::writeCameras()
{

}

void ColladaExporter::writeLights()
{
    //fs << "   <library_lights>\n";

    //fs << "   </library_lights>\n";
}

void ColladaExporter::writeImages()
{

}

void ColladaExporter::writeEffects()
{
    fs << "   <library_effects>\n";
    fs << "      <effect id=\"CapturedObject-effect\">\n";
    fs << "         <profile_COMMON>\n";
    fs << "            <technique sid=\"common\">\n";
    fs << "               <phong>\n";
    fs << "                  <emission>\n";
    fs << "                     <color>0 0 0 1</color>\n";
    fs << "                  </emission>\n";
    fs << "                  <ambient>\n";
    fs << "                     <color>0 0 0 1</color>\n";
    fs << "                  </ambient>\n";
    fs << "                  <diffuse>\n";
    fs << "                     <color>0.64 0.64 0.64 1</color>\n";
    fs << "                  </diffuse>\n";
    fs << "                  <specular>\n";
    fs << "                     <color>0.5 0.5 0.5 1</color>\n";
    fs << "                  </specular>\n";

//    fs << "                  <shininess>\n";
//    fs << "                     <float>16</float>\n";
//    fs << "                  </shininess>\n";
//    fs << "                  <reflective>\n";
//    fs << "                     <color>0 0 0 1</color>\n";
//    fs << "                  </reflective>\n";
//    fs << "                  <reflectivity>\n";
//    fs << "                     <float>0.5</float>\n";
//    fs << "                  </reflectivity>\n";
//    fs << "                  <transparent>\n";
//    fs << "                     <color>0 0 0 1</color>\n";
//    fs << "                  </transparent>\n";
//    fs << "                  <transparency>\n";
//    fs << "                     <float>1</float>\n";
//    fs << "                  </transparency>\n";
//    fs << "                  <index_of_refraction>\n";
//    fs << "                     <float>0</float>\n";
//    fs << "                  </index_of_refraction>\n";

    fs << "               </phong>\n";
    fs << "            </technique>\n";

    fs << "            <extra>\n";
    fs << "               <technique profile=\"GOOGLEEARTH\">\n";
    fs << "                  <double_sided>1</double_sided>\n";
    fs << "               </technique>\n";
    fs << "            </extra>\n";

    fs << "         </profile_COMMON>\n";

    fs << "         <extra>\n";
    fs << "            <technique profile=\"MAX3D\">\n";
    fs << "               <double_sided>1</double_sided>\n";
    fs << "            </technique>\n";
    fs << "         </extra>\n";

    fs << "      </effect>\n";
    fs << "   </library_effects>\n";
}

void ColladaExporter::writeMaterials()
{
    fs << "   <library_materials>\n";
    fs << "      <material id=\"CapturedObject-material\" name=\"Material\">\n";
    fs << "         <instance_effect url=\"#CapturedObject-effect\"/>\n";
    fs << "      </material>\n";
    fs << "   </library_materials>\n";
}

void ColladaExporter::writeGeometry()
{
    int nr_points  = input_mesh_->cloud.width * input_mesh_->cloud.height;
    //std::cout << "cloud.width: " << input_mesh_->cloud.width << "\ncloud.height: "<< input_mesh_->cloud.height << "\n";
    int point_size = input_mesh_->cloud.data.size () / nr_points;

    fs << "      <library_geometries>\n";
    fs << "         <geometry id=\"CapturedObject\">\n";
    fs << "            <mesh>\n";
    fs << "               <source id=\"CapturedObject-positions\">\n";
    fs << "                  <float_array id=\"CapturedObject-positions-array\" count=\"" << nr_points * 3 << "\">\n";

    // Export the points
    for (int i = 0; i < nr_points; ++i)
    {
        int xyz = 0;
        fs << "                    ";
        for (size_t d = 0; d < input_mesh_->cloud.fields.size (); ++d)
        {
            int c = 0;
            if ((input_mesh_->cloud.fields[d].datatype == sensor_msgs::PointField::FLOAT32) && (
                        input_mesh_->cloud.fields[d].name == "x" ||
                        input_mesh_->cloud.fields[d].name == "y" ||
                        input_mesh_->cloud.fields[d].name == "z"))
            {
                float value;
                memcpy (&value, &input_mesh_->cloud.data[i * point_size + input_mesh_->cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
                fs << " " << value;
                if (++xyz == 3)
                    break;
            }
        }
        fs << "\n";
    }

    fs << "                  </float_array>\n";
    fs << "                  <technique_common>\n";
    fs << "                     <accessor source=\"#CapturedObject-positions-array\" count=\"" << nr_points << "\" stride=\"3\">\n";
    fs << "                        <param name=\"X\" type=\"float\"/>\n";
    fs << "                        <param name=\"Y\" type=\"float\"/>\n";
    fs << "                        <param name=\"Z\" type=\"float\"/>\n";
    fs << "                     </accessor>\n";
    fs << "                  </technique_common>\n";
    fs << "               </source>\n";


    // Export normals
    //fs << "               <source id=\"CapturedObject-normals\">\n";
    //fs << "                  <float_array id=\"CapturedObject-normals-array\" count=\"" << nr_points << "\">\n";

    /// \todo export normals

    //fs << "                  </float_array>\n";
    //fs << "               </source>\n";


    // Export color data for vertices
    int field_index = -1;

    for (size_t d = 0; d < input_mesh_->cloud.fields.size (); ++d)
    {
        //std::cout << input_mesh_->cloud.fields[d].name;
        if (input_mesh_->cloud.fields[d].name == "rgb")
            field_index = (int) d;
    }

    if (field_index != -1)
    {
        fs << "               <source id=\"CapturedObject-colors\">\n";
        fs << "                  <float_array id=\"CapturedObject-colors-array\" count=\"" << nr_points * 3 << "\">\n";

        for (int i = 0; i < nr_points; ++i)
        {
            int c = 0;
            fs << "                     ";
            if (input_mesh_->cloud.fields[field_index].datatype == sensor_msgs::PointField::FLOAT32)
            {
                pcl::RGB color;
                memcpy (&color, &input_mesh_->cloud.data[i * point_size + input_mesh_->cloud.fields[field_index].offset + c * sizeof (float)], sizeof (pcl::RGB));
                int r = color.r;
                int g = color.g;
                int b = color.b;
                fs << (float)r/255.0 << " " << (float)g/255.0 << " " << (float)b/255.0;
            }
            fs << "\n";
        }

        fs << "                  </float_array>\n";
        fs << "                  <technique_common>\n";
        fs << "                     <accessor source=\"#CapturedObject-colors-array\" count=\"" << nr_points << "\" stride=\"3\">\n";
        fs << "                       <param name=\"R\" type=\"float\"/>\n";
        fs << "                       <param name=\"G\" type=\"float\"/>\n";
        fs << "                       <param name=\"B\" type=\"float\"/>\n";
        fs << "                     </accessor>\n";
        fs << "                  </technique_common>\n";
        fs << "               </source>\n";
    }

    fs << "               <vertices id=\"CapturedObject-vertices\">\n";
    fs << "                  <input semantic=\"POSITION\" source=\"#CapturedObject-positions\"/>\n";
    fs << "               </vertices>\n";

    // Calculate real value for vertices
    size_t vertices = 0;
    for (size_t i = 0; i < input_mesh_->polygons.size(); i++)
                vertices += input_mesh_->polygons[i].vertices.size();

    // Export the faces
    fs << "               <triangles count=\"" << vertices / 3 << "\" material=\"Material\">\n";
    fs << "                  <input semantic=\"VERTEX\" source=\"#CapturedObject-vertices\" offset=\"0\"/>\n";
    if (field_index != -1)
        fs << "                  <input semantic=\"COLOR\" source=\"#CapturedObject-colors\" offset=\"1\"/>\n";
    fs << "                  <p>\n";
    for (size_t i = 0; i < input_mesh_->polygons.size(); i++)
    {
        fs << "                    ";
        for (size_t j = 0; j < input_mesh_->polygons[i].vertices.size(); j++)
        {
            fs << " " << input_mesh_->polygons[i].vertices[j] << " " << input_mesh_->polygons[i].vertices[j];
        }
        fs << "\n";
    }

    fs << "                  </p>\n";
    fs << "               </triangles>\n";
    fs << "            </mesh>\n";

    fs << "            <extra>\n";
    fs << "               <technique profile=\"MAYA\">\n";
    fs << "                  <double_sided>1</double_sided>\n";
    fs << "               </technique>\n";
    fs << "            </extra>\n";

    fs << "         </geometry>\n";
    fs << "      </library_geometries>\n";
}

void ColladaExporter::writeControllers()
{
    //fs << "   <library_controllers>\n";

    //fs << "   </library_controllers>\n";
}

void ColladaExporter::writeVisualScenes()
{
    fs << "   <library_visual_scenes>\n";
    fs << "      <visual_scene id=\"Scene\" name=\"Scene\">\n";
    fs << "         <node id=\"CapturedObject-node\" type=\"NODE\">\n";
    fs << "            <translate sid=\"location\">0 0 0</translate>\n";
    fs << "            <rotate sid=\"rotationZ\">0 0 1 0</rotate>\n";
    fs << "            <rotate sid=\"rotationY\">0 1 0 0</rotate>\n";
    fs << "            <rotate sid=\"rotationX\">1 0 0 0</rotate>\n";
    fs << "            <scale sid=\"scale\">1 1 1</scale>\n";
    fs << "            <instance_geometry url=\"#CapturedObject\">\n";

    fs << "               <bind_material>\n";
    fs << "                  <technique_common>\n";
    fs << "                     <instance_material symbol=\"Material\" target=\"#CapturedObject-material\"/>\n";
    fs << "                  </technique_common>\n";
    fs << "               </bind_material>\n";
    fs << "            </instance_geometry>\n";

    fs << "         </node>\n";
    fs << "      </visual_scene>\n";
    fs << "   </library_visual_scenes>\n";
}

void ColladaExporter::writeScene()
{
    fs << "   <scene>\n";
    fs << "      <instance_visual_scene url=\"#Scene\"/>\n";
    fs << "   </scene>\n";
}

} // End of namespace ObjectCapture
