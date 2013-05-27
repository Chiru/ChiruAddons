/*
 * __API.cpp: ctypes API for python logic code
 * -----------------------------------------------------
 * This is the API for external python code to invoke internal Benchmark API
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "Neocortex_Version.h"
#include "Neocortex_API.h"

#include "Scenegraph.h"

#include "ResourceManager.h"
#include "DebugLog.h"

#include <stdio.h>
#include <math.h>

#ifdef WIN32
#ifndef M_PI
#define M_PI 3.14159265f
#endif
#endif

/******************************************************************************
 * Scenegraph manipulation API
 */

int SMG_init(void)
{
    printf("Neocortex 3D (v%d.%d.%d build %d)", MAJOR, MINOR, PATCH, BUILD);
#ifdef _DEBUG
    printf(" (DEBUG)");
#endif
    printf("\n");
    return Scenegraph::Instance()->init();
}

int SMG_destroy()
{
    return Scenegraph::Instance()->destroy();
}

int SMG_setVerbosity(unsigned int level)
{
    DebugLog::Instance()->setVerbosityLevel(level);
    return 0;
}

float SMG_getSystemtime()
{
    return Scenegraph::Instance()->getSystemtime();
}

int SMG_createDisplay(unsigned int width, unsigned int height, int fullscreen)
{
    return Scenegraph::Instance()->createDisplay(width, height, fullscreen);
}

int SMG_destroyDisplay(void)
{
    return Scenegraph::Instance()->destroyDisplay();
}

int SMG_renderFrame(float delta)
{
    return Scenegraph::Instance()->renderFrame(delta);
}

int SMG_renderFrameAndSwap(float delta)
{
    return Scenegraph::Instance()->renderFrameAndSwap(delta);
}

int SMG_swapFrame(void)
{
    return Scenegraph::Instance()->swapFrame();
}

int SMG_createObject()
{
    return Scenegraph::Instance()->createObject();
}

int SMG_destroyObject(int object_id)
{
    return Scenegraph::Instance()->destroyObject(object_id);
}

int SMG_createCamera(void)
{
    return Scenegraph::Instance()->createCamera();
}

int SMG_destroyCamera(int camera_id)
{
    return Scenegraph::Instance()->destroyCamera(camera_id);
}

int SMG_setCameraPosition(int camera_id, float x, float y, float z)
{
    return Scenegraph::Instance()->setCameraPosition(camera_id, x, y, z);
}

int SMG_setCameraPositionDelta(int camera_id, float x, float y, float z)
{
    return Scenegraph::Instance()->setCameraPositionDelta(camera_id, x, y, z);
}

int SMG_setCameraRotation(int camera_id, float x, float y, float z)
{
    return Scenegraph::Instance()->setCameraRotation(camera_id, x, y, z);
}

int SMG_setCameraRotationDegrees(int camera_id, float x, float y, float z)
{
    return SMG_setCameraRotation(camera_id, x*M_PI/180.0f, y*M_PI/180.0f, z*M_PI/180.0f);
}

int SMG_setActiveCamera(int camera_id)
{
    return Scenegraph::Instance()->setActiveCamera(camera_id);
}

int SMG_setCameraMode(int camera_id, int mode)
{
    return Scenegraph::Instance()->setCameraMode(camera_id, mode);
}

/******************************************************************************
 * Scenegraph manipulation API
 */

int OBJ_attachMesh(int object_id, int mesh_id)
{
    return Scenegraph::Instance()->attachMesh(object_id, mesh_id);
}

int OBJ_deleteMesh(int object_id)
{
    return Scenegraph::Instance()->deleteMesh(object_id);
}

int OBJ_attachMaterial(int object_id, const char *name, int submesh)
{
    return Scenegraph::Instance()->attachMaterial(object_id, name, submesh);
}

int OBJ_deleteMaterial(int object_id, int submesh)
{
    return Scenegraph::Instance()->deleteMaterial(object_id, submesh);
}

// Orientations for scenegraph objects
int OBJ_setPosition(int object_id, float x, float y, float z)
{
    return Scenegraph::Instance()->setPosition(object_id, x, y, z);
}

int OBJ_setRotation(int object_id, float x, float y, float z)
{
    return Scenegraph::Instance()->setRotation(object_id, x, y, z);
}

int OBJ_setRotationDegrees(int object_id, float x, float y, float z)
{
    return OBJ_setRotation(object_id, x*M_PI/180.0f, y*M_PI/180.0f, z*M_PI/180.0f);
}

int OBJ_setScale(int object_id, float x, float y, float z)
{
    return Scenegraph::Instance()->setScale(object_id, x, y, z);
}

int OBJ_setMeshLocalPosition(int mesh_id, float x, float y, float z)
{
    return Scenegraph::Instance()->setMeshLocalPosition(mesh_id, x, y, z);
}

int OBJ_setMeshLocalRotation(int mesh_id, float x, float y, float z)
{
    return Scenegraph::Instance()->setMeshLocalRotation(mesh_id, x, y, z);
}

int OBJ_setMeshLocalRotationDegrees(int mesh_id, float x, float y, float z)
{
    return OBJ_setMeshLocalRotation(mesh_id, x*M_PI/180.0f, y*M_PI/180.0f, z*M_PI/180.0f);
}

int OBJ_setMeshLocalScale(int mesh_id, float x, float y, float z)
{
    return Scenegraph::Instance()->setMeshLocalScale(mesh_id, x, y, z);
}

// Attributes of scenegraph objects
int OBJ_setDrawmode(int object_id, unsigned int mode)
{
    return Scenegraph::Instance()->setDrawmode(object_id, mode);
}



/******************************************************************************
 * Resource management, for creating resources used by Scenegraph objects
 */

int RES_createResource(int resource_type, const char *filename)
{
    return ResourceManager::Instance()->createResource((RES_RESOURCE_TYPE)resource_type, filename);
}

int RES_deleteResource(int resource_id)
{
    return ResourceManager::Instance()->deleteResource(resource_id);
}
