
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef Scenegraph_H
#define Scenegraph_H

#include "Neocortex_GLHeaders.h"

#include <string>
#include <vector>
#include <map>

#include "GLWrapper.h"
#include "ScenegraphObject.h"
#include "ScenegraphCamera.h"
#include "ResourceManager.h"

#include "NeocortexDisplay.h"
#include "Quadtree.h"


#ifndef WIN32
#include <sys/time.h>
#else

#define WIN32_LEAN_AND_MEAN
#include <time.h>
#include <windows.h>
#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
  #define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
#else
  #define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
#endif

struct timezone
{
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};

#endif // WIN32

typedef enum {
    SC_FEATURE_TEXTURING_ETC1 = 1,
    SC_FEATURE_TEXTURING_ETC2,
    SC_FEATURE_TEXTURING_DXT1,
    SC_FEATURE_TEXTURING_DXT1A,
    SC_FEATURE_TEXTURING_DXT3,
    SC_FEATURE_TEXTURING_DXT5,
    SC_FEATURE_32BITINDICES,
    SC_FEATURE_HALFFLOAT
} SC_FEATURE;

/// Class definition
class Scenegraph
{
public:
    static Scenegraph * Instance();
    ~Scenegraph();

    /// Global initialisation and destruction:
    int init(void);
    int destroy(void);

    /// Creates a rendering surface for the Scenegraph
    /// @return 0 on success, else an error code
    int createDisplay(unsigned int width, unsigned int height, bool fullscreen);

    /// Destroy a previously created rendering surface
    /// @return 0 on success, else an error code
    int destroyDisplay(void);

    /// Timer methods
    float getSystemtime(void);

    /// Method for querying available GL features runtime
    bool queryFeature(SC_FEATURE feat);

    /// Camera methods
    int createCamera(void);
    int destroyCamera(int camera_id);

    int setCameraPosition(float x, float y, float z);
    int setCameraPosition(int camera_id, float x, float y, float z);
    int setCameraPositionDelta(float x, float y, float z);
    int setCameraPositionDelta(int camera_id, float x, float y, float z);

    int setCameraRotation(float x, float y, float z);
    int setCameraRotation(int camera_id, float x, float y, float z);
    int setCameraRotationDelta(float x, float y, float z);
    int setCameraRotationDelta(int camera_id, float x, float y, float z);

    int setCameraMode(int camera_id, int mode);
    int setCameraMode(int mode);
    int setActiveCamera(int camera_id);

    /// Object methods
    int createObject(void);
    int destroyObject(int object_id);
    int getObjectPosition(int object_id, float *x, float *y, float *z);

    // object manipulation
    int attachMesh(int object_id, int mesh_id);
    int deleteMesh(int object_id);
    int attachMaterial(int object_id, const char *mat, int submesh);
    int deleteMaterial(int object_id, int submesh);
    // Orientations for scenegraph objects
    int setPosition(int object_id, float x, float y, float z);
    int setRotation(int object_id, float x, float y, float z);
    int setScale(int object_id, float x, float y, float z);
    // Local Orientations for meshes
    int setMeshLocalPosition(int mesh_id, float x, float y, float z);
    int setMeshLocalRotation(int mesh_id, float x, float y, float z);
    int setMeshLocalScale(int mesh_id, float x, float y, float z);
    // Attributes of scenegraph objects
    int setDrawmode(int object_id, unsigned int mode);

    int renderFrame(float delta);
    int renderFrameAndSwap(float delta);
    int swapFrame(void);

protected:

private:
    /// Scenegraph singleton
    Scenegraph() :
        currentcamera(-1),
        display(NULL),
        feat_texture_etc1(false),
        feat_texture_etc2(false),
        feat_texture_dxt1(false),
        feat_texture_dxt1a(false),
        feat_texture_dxt3(false),
        feat_texture_dxt5(false),
        feat_32bitindices(false),
        feat_halffloats(false)
        {
            init();
        }                                             // Private constructor
    Scenegraph(Scenegraph const &) {}
    static Scenegraph *p_Instance;                    // Single instance placeholder

    /// Detect hardware capabilities, and set runtime flags accordingly
    int detectGLCapabilities(void);
    void queryCompressedTextureformats(void);

    /// Quadtree container:
    Quadtree *qTree;

    /// A map containing all objects in the current scenegraph
    std::map <int, ScenegraphObject *> objects;

    /// A map containing all cameras in the current scenegraph
    std::map <int, ScenegraphCamera *> cameras;

    /// Currently active camera ID
    int currentcamera;

    /// A pointer to NeocortexDisplay class
    NeocortexDisplay *display;

    /// Private timer methods and datastructures:
    void resetSystemtime(void);
    struct timeval t_start, t_now, t_lastframe;
    struct timezone tz;

    /// Hardware features
    bool feat_texture_etc1;
    bool feat_texture_etc2;
    bool feat_texture_dxt1;
    bool feat_texture_dxt1a;
    bool feat_texture_dxt3;
    bool feat_texture_dxt5;
    bool feat_32bitindices;
    bool feat_halffloats;
};

#endif // Scenegraph_H

