/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "Scenegraph.h"
#include "ScenegraphCamera.h"
#include "ResourceManager.h"
#include "GLWrapper.h"
#include "DebugLog.h"
#include "GLMath.h"
#include "UUID.h"
#include "Quadtree.h"

#include "NeocortexDisplay.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>
#include <math.h>

Scenegraph * Scenegraph::p_Instance = NULL;

Scenegraph * Scenegraph::Instance()
{
    if (p_Instance == NULL)
    {
        p_Instance = new Scenegraph();
    }
    return p_Instance;
}

Scenegraph::~Scenegraph()
{
    destroy();
}

int Scenegraph::init(void)
{
    resetSystemtime();
    //qTree = new Quadtree(self);
    return 0;
}

int Scenegraph::destroy(void)
{
    if (display != NULL)
    {
        display->destroyDisplay();
        display = NULL;
    }
    objects.clear();
    cameras.clear();
    GFXDEBUG_PRINTSTATS();
    return 0;
}

/******************************************************************************
 * Scenegraph display manipulation API
 */

int Scenegraph::detectGLCapabilities(void)
{
    unsigned int i;
    const GLubyte *s;
#ifdef _DEBUG
    const char *tags[] = {
        "GL_VENDOR",
        "GL_RENDERER",
        "GL_VERSION",
        "GL_SHADING_LANGUAGE_VERSION",
        "GL_EXTENSIONS"
    };
#endif
    const int ids[] = {
        GL_VENDOR,
        GL_RENDERER,
        GL_VERSION,
        GL_SHADING_LANGUAGE_VERSION,
        GL_EXTENSIONS
    };

    for (i=0; i<sizeof(ids)/sizeof(int); i++)
    {
        s = GLWrapper::Instance()->GLGETSTRING(ids[i]);
        DEBUG_INFO("%s: %s\n", tags[i], s);
    }

    // Next parse the extensions string
    if (NULL != strstr((char *)s, "GL_OES_compressed_ETC1_RGB8_texture"))
    {
        DEBUG_INFO("ETC1 texturing supported\n");
        feat_texture_etc1 = true;
    }
    if (NULL != strstr((char *)s, "GL_OES_element_index_uint"))
    {
        DEBUG_INFO("32bit indices supported\n");
        feat_32bitindices = true;
    }
    if (NULL != strstr((char *)s, "GL_OES_vertex_half_float"))
    {
        DEBUG_INFO("vertex halffloats supported\n");
        feat_halffloats = true;
    }
    if (NULL != strstr((char *)s, "GL_EXT_texture_compression_dxt1"))
    {
        DEBUG_INFO("DXT1 compression supported\n");
        feat_texture_dxt1 = true;
    }
    if (NULL != strstr((char *)s, "GL_ANGLE_texture_compression_dxt5"))
    {
        DEBUG_INFO("DXT5 compression supported\n");
        feat_texture_dxt5 = true;
    }

    queryCompressedTextureformats();

    return 0;
}

void Scenegraph::queryCompressedTextureformats(void)
{
    GLint *v;
    GLint t;

    // First we query the list of supported compressed texture formats
    GLWrapper::Instance()->GLGETINTEGERV(GL_NUM_COMPRESSED_TEXTURE_FORMATS, &t);
    DEBUG_INFO("Number of compressed texture formats supported by the driver: %d\n", t);
    if (t == 0)
    {
        DEBUG_INFO("Error: The GL driver does not support texture compression.\n");
        return;
    }

    v = new GLint [t];

    GLWrapper::Instance()->GLGETINTEGERV(GL_COMPRESSED_TEXTURE_FORMATS, v);

    DEBUG_INFO("Supported compressed texture formats are:\n");
    for (int i=0; i<t; i++)
    {
        switch(v[i])
        {
        case GL_COMPRESSED_RGB_S3TC_DXT1_EXT: /* 0x83f0 */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_RGB_S3TC_DXT1_EXT)\n", i, v[i]);
            feat_texture_dxt1 = true;
            break;
        case GL_COMPRESSED_RGBA_S3TC_DXT1_EXT: /* 0x83f1 */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_RGBA_S3TC_DXT1_EXT)\n", i, v[i]);
            feat_texture_dxt1a = true;
            break;
#if defined(NC_EGL)
        /* These are compressed texture flags, which are not defined for EGL/GLES2, but may still
           be defined by the hardware. That is why they are referenced via hardcoded
           values */
        case 0x83f2: // GL_COMPRESSED_RGBA_S3TC_DXT3_EXT
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_RGBA_S3TC_DXT3_EXT)\n", i, v[i]);
            feat_texture_dxt3 = true;
            break;
        case 0x83f3: // GL_COMPRESSED_RGBA_S3TC_DXT5_EXT
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_RGBA_S3TC_DXT5_EXT)\n", i, v[i]);
            feat_texture_dxt5 = true;
            break;
        case 0x8c70: // GL_COMPRESSED_LUMINANCE_LATC1_EXT
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_LUMINANCE_LATC1_EXT)\n", i, v[i]);
            break;
        case 0x8c71: // GL_COMPRESSED_SIGNED_LUMINANCE_LATC1_EXT
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_SIGNED_LUMINANCE_LATC1_EXT)\n", i, v[i]);
            break;
        case 0x8c72: // GL_COMPRESSED_LUMINANCE_ALPHA_LATC2_EXT
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_LUMINANCE_ALPHA_LATC2_EXT)\n", i, v[i]);
            break;
        case 0x8c73: // GL_COMPRESSED_SIGNED_LUMINANCE_ALPHA_LATC2_EXT
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_SIGNED_LUMINANCE_ALPHA_LATC1_EXT)\n", i, v[i]);
            break;
#endif
#if !defined(NC_GLX)
        case GL_ETC1_RGB8_OES: /* 0x8d64 */
            DEBUG_INFO("format %d: 0x%x (GL_ETC1_RGB8_OES)\n", i, v[i]);
            feat_texture_etc1 = true;
            break;
#endif
#if !defined(NC_EGL)
        case GL_COMPRESSED_RGBA_S3TC_DXT3_EXT: /* 0x83f2 */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_RGBA_S3TC_DXT3_EXT)\n", i, v[i]);
            feat_texture_dxt3 = true;
            break;
        case GL_COMPRESSED_RGBA_S3TC_DXT5_EXT: /* 0x83f3 */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_RGBA_S3TC_DXT5_EXT)\n", i, v[i]);
            feat_texture_dxt5 = true;
            break;
        case GL_COMPRESSED_LUMINANCE_LATC1_EXT: /* 0x8c70 */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_LUMINANCE_LATC1_EXT)\n", i, v[i]);
            break;
        case GL_COMPRESSED_SIGNED_LUMINANCE_LATC1_EXT: /* 0x8c71 */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_SIGNED_LUMINANCE_LATC1_EXT)\n", i, v[i]);
            break;
        case GL_COMPRESSED_LUMINANCE_ALPHA_LATC2_EXT: /* 0x8c72 */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_LUMINANCE_ALPHA_LATC2_EXT)\n", i, v[i]);
            break;
        case GL_COMPRESSED_SIGNED_LUMINANCE_ALPHA_LATC2_EXT: /* 0x8c73 */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_SIGNED_LUMINANCE_ALPHA_LATC1_EXT)\n", i, v[i]);
            break;
        case GL_COMPRESSED_SRGB_S3TC_DXT1_EXT: /* 0x8c4c */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_SRGB_S3TC_DXT1_EXT\n", i, v[i]);
            break;
        case GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT: /* 0x8c4d */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT)\n", i, v[i]);
            break;
        case GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT: /* 0x8c4e */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT)\n", i, v[i]);
            break;
        case GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT: /* 0x8c4f */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT)\n", i, v[i]);
            break;
        case GL_COMPRESSED_RGB_FXT1_3DFX: /* 0x86b0 */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_RGB_FXT1_3DFX)\n", i, v[i]);
            break;
        case GL_COMPRESSED_RGBA_FXT1_3DFX: /* 0x86b1 */
            DEBUG_INFO("format %d: 0x%x (GL_COMPRESSED_RGBA_FXT1_3DFX)\n", i, v[i]);
            break;
#endif
        default:
            DEBUG_INFO("format %d: 0x%x (UNKNOWN FORMAT)\n", i, v[i]);
            break;
        }
    }
    delete v;
}

bool Scenegraph::queryFeature(SC_FEATURE feat)
{
    switch (feat)
    {
    case SC_FEATURE_TEXTURING_ETC1:
        return feat_texture_etc1;
    case SC_FEATURE_TEXTURING_ETC2:
        return feat_texture_etc2;
    case SC_FEATURE_TEXTURING_DXT1:
        return feat_texture_dxt1;
    case SC_FEATURE_TEXTURING_DXT1A:
        return feat_texture_dxt1a;
    case SC_FEATURE_TEXTURING_DXT3:
        return feat_texture_dxt3;
    case SC_FEATURE_TEXTURING_DXT5:
        return feat_texture_dxt5;
    case SC_FEATURE_32BITINDICES:
        return feat_32bitindices;
    case SC_FEATURE_HALFFLOAT:
        return feat_halffloats;
    }
    return false;
}

int Scenegraph::createDisplay(unsigned int width, unsigned int height, bool fullscreen)
{
    int rc;
    if (display == NULL)
    {
        // First time run, create the display instance
        display = new NeocortexDisplay();
    }
    else
    {
        // Class instance already exists, destroy possible previous display
        display->destroyDisplay();
    }
    // And finally create the display
    rc = display->createDisplay(width, height, fullscreen);

    // If display open was successful, then init basic GL
    if (rc == 0)
    {
        detectGLCapabilities();
        GLWrapper::Instance()->GLCLEARCOLOR(0, 0, 0, 0);
        GLWrapper::Instance()->GLVIEWPORT(0, 0, width, height);
        GLWrapper::Instance()->GLENABLE(GL_DEPTH_TEST);
        GLWrapper::Instance()->GLENABLE(GL_CULL_FACE);
        GLWrapper::Instance()->GLENABLE(GL_BLEND);
        GLWrapper::Instance()->GLBLENDFUNC(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    return rc;
}

int Scenegraph::destroyDisplay(void)
{
    int rc;
    if (display == NULL) return -1;
    rc = display->destroyDisplay();
    delete display;
    display = NULL;
    return rc;
}

/******************************************************************************
 * Scenegraph timers
 */

void Scenegraph::resetSystemtime()
{
    gettimeofday( &t_start , &tz);
    t_now = t_lastframe = t_start;
}

float Scenegraph::getSystemtime()
{
    gettimeofday(&t_now, &tz);
    return (float)(t_now.tv_sec - t_start.tv_sec + (t_now.tv_usec - t_start.tv_usec) * 1e-6);
}

/******************************************************************************
 * Scenegraph camera manipulation API
 */

int Scenegraph::createCamera(void)
{
    ScenegraphCamera *c;
    c = new ScenegraphCamera(UUID::Instance()->getUUID());
    cameras.insert(std::pair<int, ScenegraphCamera *>(c->getID(), c));
    currentcamera = c->getID();
    DEBUG_NOTIFICATION("Created new scenegraph camera with ID=%d\n", c->getID());

    c->setRenderdepth(1.0f, 1500.0f);
    c->setDisplayDimensions(display->getDisplayWidth(), display->getDisplayHeight());
    c->setViewangle(60.0f);

    return currentcamera;
}

int Scenegraph::destroyCamera(int camera_id)
{
    if (cameras.find(camera_id) == cameras.end())
    {
        DEBUG_NOTIFICATION("Trying to delete camera %d, which does not exist\n", camera_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Deleting camera id %d\n", camera_id);
    cameras.erase(camera_id);

    if (currentcamera == camera_id) // If the current camera was deleted, then re-select the first from the map
    {
        std::map <int, ScenegraphCamera *>::iterator it = cameras.begin();
        if (it == cameras.end()) // No more cameras in the array
        {
            currentcamera = -1;
            return 0;
        }
        currentcamera = it->first;
    }
    return 0;
}

int Scenegraph::setActiveCamera(int camera_id)
{
    if (cameras.find(camera_id) == cameras.end())
    {
        DEBUG_NOTIFICATION("Trying to set camera %d, which does not exist\n", camera_id);
        return -1;
    }
    currentcamera = camera_id;
    return 0;
}

int Scenegraph::setCameraPosition(float x, float y, float z)
{
    return setCameraPosition(currentcamera, x, y, z);
}

int Scenegraph::setCameraPosition(int camera_id, GLfloat x, GLfloat y, GLfloat z)
{
    std::map <int, ScenegraphCamera *>::iterator it;

    it = cameras.find(camera_id);
    if (it == cameras.end())
    {
        DEBUG_NOTIFICATION("Trying to set camera %d, which does not exist\n", camera_id);
        return -1;
    }
    return (*it).second->setCameraPosition(x, y, z);
}

int Scenegraph::setCameraPositionDelta(float x, float y, float z)
{
    return setCameraPositionDelta(currentcamera, x, y, z);
}

int Scenegraph::setCameraPositionDelta(int camera_id, GLfloat x, GLfloat y, GLfloat z)
{
    std::map <int, ScenegraphCamera *>::iterator it;

    it = cameras.find(camera_id);
    if (it == cameras.end())
    {
        DEBUG_NOTIFICATION("Trying to set camera %d, which does not exist\n", camera_id);
        return -1;
    }
    return (*it).second->setCameraPositionDelta(x, y, z);
}

int Scenegraph::setCameraRotation(float x, float y, float z)
{
    return setCameraRotation(currentcamera, x, y, z);
}

int Scenegraph::setCameraRotation(int camera_id, GLfloat x, GLfloat y, GLfloat z)
{
    std::map <int, ScenegraphCamera *>::iterator it;

    it = cameras.find(camera_id);
    if (it == cameras.end())
    {
        DEBUG_NOTIFICATION("Trying to set camera %d, which does not exist\n", camera_id);
        return -1;
    }
    return (*it).second->setCameraRotation(x, y, z);
}

int Scenegraph::setCameraRotationDelta(float x, float y, float z)
{
    return setCameraRotationDelta(currentcamera, x, y, z);
}

int Scenegraph::setCameraRotationDelta(int camera_id, GLfloat x, GLfloat y, GLfloat z)
{
    std::map <int, ScenegraphCamera *>::iterator it;

    it = cameras.find(camera_id);
    if (it == cameras.end())
    {
        DEBUG_NOTIFICATION("Trying to set camera %d, which does not exist\n", camera_id);
        return -1;
    }
    return (*it).second->setCameraRotationDelta(x, y, z);
}

int Scenegraph::setCameraMode(int mode)
{
    return setCameraMode(currentcamera, mode);
}

int Scenegraph::setCameraMode(int camera_id, int mode)
{
    std::map <int, ScenegraphCamera *>::iterator it;

    it = cameras.find(camera_id);
    if (it == cameras.end())
    {
        DEBUG_NOTIFICATION("Trying to set mode for camera %d, which does not exist\n", camera_id);
        return -1;
    }
    return (*it).second->setCameraMode((CAMERA_MODE)mode);
}

/******************************************************************************
 * Scenegraph object creation API
 */

int Scenegraph::createObject(void)
{
    ScenegraphObject *obj = new ScenegraphObject(UUID::Instance()->getUUID());
    DEBUG_NOTIFICATION("Creating new Scenegraph object with ID %d\n", obj->getID());
    objects.insert(std::pair<int, ScenegraphObject *>(obj->getID(), obj));
    return obj->getID();
}

int Scenegraph::destroyObject(int object_id)
{
    if (objects.find(object_id) == objects.end())
    {
        DEBUG_NOTIFICATION("Trying to delete object %d, which does not exist\n", object_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Deleting object ID %d\n", object_id);
    objects.erase(object_id);
    return 0;
}

int Scenegraph::getObjectPosition(int object_id, float *x, float *y, float *z)
{
    ScenegraphObject *o;
    if (objects.find(object_id) == objects.end())
    {
        DEBUG_NOTIFICATION("Trying to read position of an object %d, which does not exist\n", object_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Reading object ID %d position\n", object_id);
    o = objects[object_id];
    return o->getPosition(x, y, z);
}

/******************************************************************************
 * Scenegraph object manipulation API
 */

int Scenegraph::attachMesh(int object_id, int mesh_id)
{
    ScenegraphObject *o;
    if (objects.find(object_id) == objects.end())
    {
        DEBUG_NOTIFICATION("Trying to attach a mesh to object ID %d, which does not exist\n", object_id);
        return -1;
    }
    DEBUG_NOTIFICATION("attaching mesh %d to object %d\n", mesh_id, object_id);
    o = objects[object_id];
    return o->attachMesh(mesh_id);
}

int Scenegraph::deleteMesh(int object_id)
{
    ScenegraphObject *o;
    if (objects.find(object_id) == objects.end())
    {
        DEBUG_NOTIFICATION("Trying to delete a mesh from object ID %d, which does not exist\n", object_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Deleting mesh from object %d\n", object_id);
    o = objects[object_id];
    return o->deleteMesh();
}

int Scenegraph::attachMaterial(int object_id, const char *mat, int submesh)
{
    ScenegraphObject *o;
    if (objects.find(object_id) == objects.end())
    {
        DEBUG_NOTIFICATION("Trying to attach a material to object ID %d, which does not exist\n", object_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Attaching material %s to object %d, submesh %d\n", mat, object_id, submesh);
    o = objects[object_id];
    return o->attachMaterial(mat, submesh);
}

int Scenegraph::deleteMaterial(int object_id, int submesh)
{
    ScenegraphObject *o;
    if (objects.find(object_id) == objects.end())
    {
        DEBUG_NOTIFICATION("Trying to delete a material from object ID %d, which does not exist\n", object_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Deleting material from object %d. submesh %d\n", object_id, submesh);
    o = objects[object_id];
    return o->deleteMaterial(submesh);
}

// Orientations for scenegraph objects
int Scenegraph::setPosition(int object_id, float x, float y, float z)
{
    ScenegraphObject *o;
    if (objects.find(object_id) == objects.end())
    {
        DEBUG_NOTIFICATION("Trying to set a position to an object ID %d, which does not exist\n\n", object_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Setting a position to an object %d (%f, %f, %f)\n", object_id, x, y, z);
    o = objects[object_id];
    return o->setPosition(x, y, z);
}

int Scenegraph::setRotation(int object_id, float x, float y, float z)
{
    ScenegraphObject *o;
    if (objects.find(object_id) == objects.end())
    {
        DEBUG_NOTIFICATION("Trying to set a rotation to an object ID %d, which does not exist\n\n", object_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Setting a rotation to an object %d (%f, %f, %f)\n", object_id, x, y, z);
    o = objects[object_id];
    return o->setRotation(x, y, z);
}

int Scenegraph::setScale(int object_id, float x, float y, float z)
{
    ScenegraphObject *o;
    if (objects.find(object_id) == objects.end())
    {
        DEBUG_NOTIFICATION("Trying to set a scale to an object ID %d, which does not exist\n\n", object_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Setting a scale to an object %d (%f, %f, %f)\n", object_id, x, y, z);
    o = objects[object_id];
    return o->setScale(x, y, z);
}

// Orientations for scenegraph objects
int Scenegraph::setMeshLocalPosition(int mesh_id, float x, float y, float z)
{
    RMesh *m;
    m = dynamic_cast<RMesh *>(ResourceManager::Instance()->getResource(mesh_id));
    if (m == NULL)
    {
        DEBUG_WARNING("Trying to set mesh position for mesh %d, but it does not exist\n", mesh_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Setting local position for mesh %d, %f %f %f\n", mesh_id, x, y, z);
    return m->setLocalPosition(x, y, z);
}

int Scenegraph::setMeshLocalRotation(int mesh_id, float x, float y, float z)
{
    RMesh *m;
    m = dynamic_cast<RMesh *>(ResourceManager::Instance()->getResource(mesh_id));
    if (m == NULL)
    {
        DEBUG_WARNING("Trying to set mesh rotation for mesh %d, but it does not exist\n", mesh_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Setting local rotation for mesh %d, %f %f %f\n", mesh_id, x, y, z);
    return m->setLocalRotation(x, y, z);
}

int Scenegraph::setMeshLocalScale(int mesh_id, float x, float y, float z)
{
    RMesh *m;
    m = dynamic_cast<RMesh *>(ResourceManager::Instance()->getResource(mesh_id));
    if (m == NULL)
    {
        DEBUG_WARNING("Trying to set mesh scale for mesh %d, but it does not exist\n", mesh_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Setting local scale for mesh %d, %f %f %f\n", mesh_id, x, y, z);
    return m->setLocalScale(x, y, z);
}

int Scenegraph::setDrawmode(int object_id, unsigned int mode)
{
    ScenegraphObject *o;
    if (objects.find(object_id) == objects.end())
    {
        DEBUG_NOTIFICATION("Trying to set a drawmode to an object ID %d, which does not exist\n\n", object_id);
        return -1;
    }
    DEBUG_NOTIFICATION("Setting a drawmode to an object %d, mode %d\n", object_id, mode);
    o = objects[object_id];
    return o->setDrawmode(mode);
}

/******************************************************************************
 * Rendering
 */

#define CAMERA_DELTA (1.0f)

int Scenegraph::renderFrameAndSwap(float delta)
{
    if (-1 == renderFrame(delta)) return -1;
    return swapFrame();
}

int Scenegraph::swapFrame(void)
{
    return display->swapBuffers();
}

int Scenegraph::renderFrame(float delta)
{
    static int mouse_button = 0;
    static int mouse_x, mouse_y;
    ScenegraphCamera *c;
    EGLX11EVENT event;

    GFXDEBUG_RESET();

#ifdef _DEBUG
    float t1, t2;
    t1 = getSystemtime();
#endif

    DEBUG_INFO("Scenegraph render delta=%f\n", delta);

    if (cameras.find(currentcamera) == cameras.end())
    {
        DEBUG_NOTIFICATION("No camera defined. Aborting!\n");
        return -1;
    }
    c = cameras[currentcamera];

#ifdef _DEBUG
    t2 = getSystemtime();
    DEBUG_INFO("Profile: cameras.find %f seconds\n", t2-t1);
    t1 = t2;
#endif

    while (display->eventPump(&event))
    {
        switch (event.event)
        {
        case EVENT_KEYEVENT:
            DEBUG_FATAL("Keypress detected, key %c (%d 0x%x)\n",
                               event.u.keyEvent.key,
                               event.u.keyEvent.key,
                               event.u.keyEvent.key);
            switch(event.u.keyEvent.key)
            {
            case 'w':
                c->setCameraPositionDelta(0.0f, 0.0f, -CAMERA_DELTA);
                break;
            case 'a':
                c->setCameraPositionDelta(-CAMERA_DELTA, 0.0f, 0.0f);
                break;
            case 's':
                c->setCameraPositionDelta(0.0f, 0.0f, CAMERA_DELTA);
                break;
            case 'd':
                c->setCameraPositionDelta(CAMERA_DELTA, 0.0f, 0.0f);
                break;
            case 'r':
                c->setCameraPositionDelta(0.0f, CAMERA_DELTA, 0.0f);
                break;
            case 'f':
                c->setCameraPositionDelta(0.0f, -CAMERA_DELTA, 0.0f);
                break;
            case 'z':
                c->setCameraRotationDelta(0.0f, -0.05f, 0.0f);
                break;
            case 'x':
                c->setCameraRotationDelta(0.0f, -0.05f, 0.0f);
                break;
            case 27:
                DEBUG_NOTIFICATION("Escape pressed. Aborting\n");
                return -1;
            }
#if 1
            {
                float x, y, z;
                c->getCameraPosition(&x, &y, &z);
                DEBUG_FATAL("Current camera pos: %f %f %f\n", x, y, z);
                c->getCameraRotation(&x, &y, &z);
                DEBUG_FATAL("Current camera rot: %f %f %f\n", x/3.1415f*180.0f, y/3.1415f*180.0f, z/3.1415f*180.0f);
            }
#endif
            break;
        case EVENT_MOUSEBUTTONEVENT:
            DEBUG_FATAL("Mouse button event, status %d, button id %d\n", event.u.mouseButtonEvent.status, event.u.mouseButtonEvent.button);
            if (event.u.mouseButtonEvent.button == 3 && event.u.mouseButtonEvent.status == true)
            {
                mouse_button = 1;
                mouse_x = event.u.mouseButtonEvent.x;
                mouse_y = event.u.mouseButtonEvent.y;
            }
            else mouse_button = 0;
            break;
        case EVENT_MOUSEMOTIONEVENT:
            if (mouse_button == 0) break;
            //DEBUG_FATAL("Mouse motion X %d Y %d - delta X %d Y %d\n", event.u.mouseMotionEvent.x, event.u.mouseMotionEvent.y,
            //            event.u.mouseMotionEvent.x-mouse_x, event.u.mouseMotionEvent.y-mouse_y);
            c->setCameraRotationDelta(0.0f, (GLfloat)(event.u.mouseMotionEvent.x-mouse_x)*3.1415f/180.0f, (GLfloat)(event.u.mouseMotionEvent.y-mouse_y)*3.1415f/180.0f);
            mouse_x = event.u.mouseMotionEvent.x;
            mouse_y = event.u.mouseMotionEvent.y;
            break;
        case EVENT_QUIT:
            DEBUG_FATAL("System quit signal received.\n");
            return -1;
        default:
            break;
        }
    }

#ifdef _DEBUG
    t2 = getSystemtime();
    DEBUG_INFO("Profile: event pump %f seconds\n", t2-t1);
    t1 = t2;
#endif

    c->applyProjectionMatrix();
    GLMath::Instance()->_glMatrixMode(_GL_MATRIXMODE_MODELVIEW);
    GLMath::Instance()->_glLoadIdentity();

#ifdef _DEBUG
    GLMath::Instance()->_glDebugMatrix(_GL_MATRIXMODE_PROJECTION);
    GLMath::Instance()->_glDebugMatrix(_GL_MATRIXMODE_MODELVIEW);
    t2 = getSystemtime();
    DEBUG_INFO("Profile: matrix formations %f seconds\n", t2-t1);
    t1 = t2;
#endif

    GLWrapper::Instance()->GLCLEAR(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

#ifdef _DEBUG
    t2 = getSystemtime();
    DEBUG_INFO("Profile: glClear %f seconds\n", t2-t1);
    t1 = t2;
#endif

    int total = 0, rendered = 0;
#if 1
    float rot_x, rot_y, rot_z;
    float cpx, cpy, cpz;
    float cx, cy, cz;
    c->getCameraRotation(&rot_x, &rot_y, &rot_z);
    c->getCameraPosition(&cpx, &cpy, &cpz);
    cx = sin(rot_y); cy=0.0f; cz=cos(rot_y);
    //printf("camera rot %f %f %f, pos %f %f %f, view %f %f %f\n", rot_x, rot_y, rot_z, cpx, cpy, cpz, cx, cy, cz);
#endif
    
    std::map <int, ScenegraphObject *>::iterator it = objects.begin();
    for (it = objects.begin(); it != objects.end(); it++)
    {
        DEBUG_INFO("Rendering object %d\n", (*it).first);
#if 1
        float x, y, z, l, dot;
        (*it).second->getPosition(&x, &y, &z);
        x = x-cpx; y = y-cpy; z = z-cpz;
        l = sqrt(x*x + y*y + z*z);
        x /= l; y /= l; z /= l;
        dot = x*cx+y*cy+z*cz;
        //printf("obj direction %f %f %f, dot camera %f\n", x, y, z, dot);
#endif
        (*it).second->isVisible();        
        if (dot < -0.707f)
        {
            GLMath::Instance()->_glPushMatrix();
            (*it).second->render(delta);
            GLMath::Instance()->_glPopMatrix();
            rendered++;
        }
        else
        {
        }
        total++;
#ifdef _DEBUG
        t2 = getSystemtime();
        DEBUG_INFO("Profile: object render %f seconds\n", t2-t1);
        t1 = t2;
#endif
    }
    DEBUG_INFO("culling Total objects per frame %d, rendered %d, hence %02.1f%% coverage\n", total, rendered, (float)100.0f*rendered/total);

    return 0;
}
