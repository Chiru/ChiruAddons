/*
 * __API.h: ctypes API for python logic code
 * -----------------------------------------------------
 * This is the API for external python code to invoke internal Benchmark API
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef NEOCORTEX_API_H
#define NEOCORTEX_API_H

#include "Neocortex_Datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * Scene graph manipulation API
 */

int SMG_init(void);
int SMG_destroy(void);
int SMG_setVerbosity(unsigned int level);
float SMG_getSystemtime(void);

int SMG_createDisplay(unsigned int width, unsigned int height, int fullscreen);
int SMG_destroyDisplay(void);
int SMG_renderFrame(float delta);
int SMG_renderFrameAndSwap(float delta);
int SMG_swapFrame(void);

int SMG_createObject(void);
int SMG_destroyObject(int object_id);

int SMG_createCamera(void);
int SMG_destroyCamera(int camera_id);
int SMG_setCameraPosition(int camera_id, float x, float y, float z);
int SMG_setCameraRotation(int camera_id, float x, float y, float z);
int SMG_setCameraRotationDegrees(int camera_id, float x, float y, float z);
int SMG_setCameraPositionDelta(int camera_id, float x, float y, float z);
int SMG_setActiveCamera(int camera_id);
int SMG_setCameraMode(int camera_id, int mode);

/******************************************************************************
 * Scene graph object manipuation API
 */

int OBJ_attachMesh(int object_id, int mesh_id);
int OBJ_deleteMesh(int object_id);
int OBJ_attachMaterial(int object_id, const char *name, int submesh);
int OBJ_deleteMaterial(int object_id, int submesh);
// Orientations for scenegraph objects
int OBJ_setPosition(int object_id, float x, float y, float z);
int OBJ_setRotation(int object_id, float x, float y, float z);
int OBJ_setRotationDegrees(int object_id, float x, float y, float z);
int OBJ_setScale(int object_id, float x, float y, float z);
// Local orientations for meshes:
int OBJ_setMeshLocalPosition(int object_id, float x, float y, float z);
int OBJ_setMeshLocalRotation(int object_id, float x, float y, float z);
int OBJ_setMeshLocalRotationDegrees(int object_id, float x, float y, float z);
int OBJ_setMeshLocalScale(int object_id, float x, float y, float z);
// Attributes of scenegraph objects
int OBJ_setDrawmode(int object_id, unsigned int mode);

/******************************************************************************
 * Resource management, for creating resources used by Scenegraph objects
 */

int RES_createResource(int resource_type, const char *filename);
int RES_deleteResource(int resource_id);

/******************************************************************************
 * I/O Management
 */

#ifdef __cplusplus
} // extern "C"
#endif

#endif // __API_H
