
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "GLWrapper.h"
#include "DebugLog.h"
#include "GLMath.h"

#include "ResourceManager.h"
#include "RShader.h"
#include "RMesh.h"
#include "RTexture.h"

#include "ScenegraphObject.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>
#include <math.h>

/******************************************************************************
 * ScenegraphObject construction and destruction
 */

ScenegraphObject::ScenegraphObject(unsigned int id) :
    mesh_id(-1),
    pos_x(0.0f),
    pos_y(0.0f),
    pos_z(0.0f),
    rot_x(0.0f),
    rot_y(0.0f),
    rot_z(0.0f),
    sca_x(1.0f),
    sca_y(1.0f),
    sca_z(1.0f)
{
    DEBUG_INFO("New Scenegraph object construction, id %d\n", id);
    ID = id;
    memset(materials, -1, sizeof(materials));
}

ScenegraphObject::~ScenegraphObject()
{
    DEBUG_INFO("Scenegraph object %d destruction\n", ID);
}


/******************************************************************************
 * Draw modes and object VBOs
 */

int ScenegraphObject::attachMesh(int id)
{
    mesh_id = id;
    return 0;
}

int ScenegraphObject::deleteMesh()
{
    mesh_id = -1;
    return 0;
}

int ScenegraphObject::attachMaterial(const char *mat, int submesh)
{
    Resource *r;
    if (submesh < 0 || submesh >= MAX_SUBMESHES)
    {
        DEBUG_WARNING("Trying to set submesh material to index (%d) not in range of 0->%d\n", submesh, MAX_SUBMESHES);
        return -1;
    }
    r = ResourceManager::Instance()->getResourceByName(mat);
    if (r == NULL)
    {
        DEBUG_WARNING("Trying to set material override '%s', which is not in resource pool. Ignoring\n", mat);
        return -1;
    }
    DEBUG_INFO("Attaching material %s (ID=%d) to submesh %d\n", mat, r->getID(), submesh);
    materials[submesh] = r->getID();
    return 0;
}

int ScenegraphObject::deleteMaterial(int submesh)
{
    if (submesh < 0 || submesh >= MAX_SUBMESHES)
    {
        DEBUG_WARNING("Trying to delete submesh material from index (%d) not in range of 0<=X<%d\n", submesh, MAX_SUBMESHES);
        return -1;
    }
    DEBUG_INFO("Deleting submesh %d material\n", submesh);
    materials[submesh] = -1;
    return 0;
}

// Orientations for scenegraph objects
int ScenegraphObject::setPosition(float x, float y, float z)
{
    pos_x = x;
    pos_y = y;
    pos_z = z;
    return 0;
}

int ScenegraphObject::setRotation(float x, float y, float z)
{
    rot_x = x;
    rot_y = y;
    rot_z = z;
    return 0;
}

int ScenegraphObject::setScale(float x, float y, float z)
{
    sca_x = x;
    sca_y = y;
    sca_z = z;
    return 0;
}

int ScenegraphObject::getPosition(float *x, float *y, float *z)
{
    *x = pos_x;
    *y = pos_y;
    *z = pos_z;
    return 0;
}

int ScenegraphObject::setDrawmode(unsigned int mode)
{
    return 0;
}

int ScenegraphObject::isVisible(void)
{
#if 1
    RMesh *mesh;
    Matrix4X4 *transform;
    GLfloat x, y, z;

    if (mesh_id == -1) return 0;
    mesh = dynamic_cast<RMesh *>(ResourceManager::Instance()->getResource(mesh_id));

    DEBUG_INFO("Applying ScenegraphObject transformation\n");
    GLMath::Instance()->_glLoadIdentity();

    mesh->getLocalPosition(&x, &y, &z);
    GLMath::Instance()->_glTranslate((pos_x+x), (pos_y+y), (pos_z+z));

    mesh->getLocalRotation(&x, &y, &z);
    GLMath::Instance()->_glRotateEuler(-(rot_x+x), -(rot_y+y), -(rot_z+z));

    mesh->getLocalScale(&x, &y, &z);
    GLMath::Instance()->_glScale(x*sca_x, y*sca_y, z*sca_z);

    return 1; // Object always visible, will return to this later

    transform = GLMath::Instance()->getTransformation();

    //float z = GLMath::MatMulVec4(transform, pos_x, pos_y, pos_z, 1.0f);
    z = transform->m[0][2]*pos_x + transform->m[1][2]*pos_y + transform->m[2][2]*pos_z + transform->m[3][2];
    DEBUG_INFO("culling %f %f %f -- %f\n", pos_x, pos_y, pos_z, z);
    if (z < 1.0f) return 0;
    return 1;
#else
    float z = transform->m[0][2]*pos_x + transform->m[1][2]*pos_y + transform->m[2][2]*pos_z + transform->m[3][2];
    DEBUG_INFO("culling %f %f %f -- %f\n", pos_x, pos_y, pos_z, z);
    if (z < 1.0f) return 0;
    return 1;
#endif
}


/******************************************************************************
 * ScenegraphObject rendering
 */

int ScenegraphObject::render(float delta)
{
    RMesh     *mesh;
    Matrix4X4 *m;

    DEBUG_INFO("Rendering scenegraph object %d\n", ID);

    if (mesh_id == -1) return 0;
    mesh = dynamic_cast<RMesh *>(ResourceManager::Instance()->getResource(mesh_id));
    m = GLMath::Instance()->getTransformation();

#ifdef _DEBUG
    DEBUG_NOTIFICATION("Projection matrix:\n");
    GLMath::Instance()->_glDebugMatrix(_GL_MATRIXMODE_PROJECTION);
    DEBUG_NOTIFICATION("Projection matrix:\n");
    GLMath::Instance()->_glDebugMatrix(_GL_MATRIXMODE_TRANSFORMATION);
#endif

    mesh->applyTransformation(m);
    mesh->render(delta, materials);
    return 0;
}
