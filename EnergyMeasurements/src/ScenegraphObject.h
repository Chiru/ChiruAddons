
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef ScenegraphObject_H
#define ScenegraphObject_H

#include "Neocortex_GLHeaders.h"

#include <string>
#include <vector>

#include "GLWrapper.h"
#include "RMesh.h"

/// Class definition
class ScenegraphObject
{
public:
    ScenegraphObject(unsigned int ID);
    ~ScenegraphObject();

    int getID(void) { return ID; }

    int attachMesh(int mesh_id);
    int deleteMesh();
    int attachMaterial(const char *mat, int submesh);
    int deleteMaterial(int submesh);

    // Orientations for scenegraph objects
    int setPosition(float x, float y, float z);
    int setRotation(float x, float y, float z);
    int setScale(float x, float y, float z);

    int getPosition(float *x, float *y, float *z);

    // Check agaist current transformation whether the object is visible
    int isVisible();

    // Attributes of scenegraph objects
    int setDrawmode(unsigned int mode);

    // Rendering
    int render(float delta);

protected:

private:
    int ID;

    // Currently attached resources.
    int mesh_id;
    int materials[MAX_SUBMESHES];

    // Current object orientation
    float pos_x, pos_y, pos_z;
    float rot_x, rot_y, rot_z;
    float sca_x, sca_y, sca_z;
};

#endif // ScenegraphObject_H
