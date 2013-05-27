
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef RMesh_H
#define RMesh_H

#include "Neocortex_GLHeaders.h"

#include <string>
#include "ResourceManager.h"
#include "RMeshContainer.h"
#include "GLWrapper.h"
#include "GLMath.h"


#define MAX_SUBMESHES       (24)
#define MAX_TEX_CHANNELS    (4)


/// Class definition

class RMesh : public Resource
{
public:
    RMesh(int id);
    ~RMesh();

    int bind(void);
    int destroy(void);
    int render(float deltatime);
    int render(float deltatime, int *materials);

    int setMeshFromFile(const char *filename);
    int setSubmeshTexture(int submesh, int channel, int texture_id);

    /// ::finalise() will optimize the loaded mesh, see documentation in the RMeshContainer API
    int finalize();

    int applyTransformation(Matrix4X4 *m);
    int applyLocalTranslation(void);
    int applyLocalRotation(void);
    int applyLocalScaling(void);

    int setLocalPosition(GLfloat x, GLfloat y, GLfloat z) {
        DEBUG_NOTIFICATION("RMesh::setPosition(%f, %f, %f)\n", x, y, z);
        pos_x = x; pos_y = y; pos_z = z; return 0;
    }
    int setLocalRotation(GLfloat x, GLfloat y, GLfloat z) {
        DEBUG_NOTIFICATION("RMesh::setRotation(%f, %f, %f)\n", x, y, z);
        rot_x = x; rot_y = y; rot_z = z; return 0;
    }
    int setLocalScale(GLfloat x, GLfloat y, GLfloat z) {
        DEBUG_NOTIFICATION("RMesh::setScale(%f, %f, %f)\n", x, y, z);
        sca_x = x; sca_y = y; sca_z = z; return 0;
    }

    int getLocalPosition(GLfloat *x, GLfloat *y, GLfloat *z) {
        *x = pos_x; *y = pos_y; *z = pos_z; return 0;
    }
    int getLocalRotation(GLfloat *x, GLfloat *y, GLfloat *z) {
        *x = rot_x; *y = rot_y; *z = rot_z; return 0;
    }
    int getLocalScale(GLfloat *x, GLfloat *y, GLfloat *z) {
        *x = sca_x; *y = sca_y; *z = sca_z; return 0;
    }

public:
    RMeshContainer * getCurrentMeshContainer(void);

private:
    int getFileType(const char *filename);

    /// OgreXML loader
    int setMeshFromOgreXML(const char *filename);
    int parseOgreXMLRootNode(void);

    /// Wavefront OBJ loader
    int setMeshFromOBJ(const char *filename);

    RMeshContainer * sharedgeometry;
    std::vector <RMeshContainer *> submeshes;
    int textures[MAX_SUBMESHES][MAX_TEX_CHANNELS];   // texture placeholders for submeshes, and texture channels
    int texture_count[MAX_SUBMESHES];

    Matrix4X4 * currentTransformation;

    // Mesh local orientation
    GLfloat pos_x, pos_y, pos_z;
    GLfloat rot_x, rot_y, rot_z;
    GLfloat sca_x, sca_y, sca_z;

    // Finalisation status
    bool finalized;
};

#endif // RMesh_H
