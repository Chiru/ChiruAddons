
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef ScenegraphCamera_H
#define ScenegraphCamera_H

#include "Neocortex_GLHeaders.h"

#include "GLWrapper.h"
#include "DebugLog.h"

typedef enum {
    MODE_WASD           = 1,
    MODE_SPHERICAL      = 2
} CAMERA_MODE;

/// Class definition
class ScenegraphCamera
{
public:
    ScenegraphCamera(int id);
    ~ScenegraphCamera();

    /// Camera view parameters
    int setViewangle(GLfloat angle) {
        DEBUG_NOTIFICATION("Setting camera view angle to %f\n", angle);
        viewangle = angle; return 0;
    }
    int setDisplayDimensions(GLfloat width, GLfloat height) {
        DEBUG_NOTIFICATION("Setting camera display dimensions to %f %f\n", width, height);
        s_width = width; s_height = height; return 0;
    }
    int setRenderdepth(GLfloat n, GLfloat f) {
        DEBUG_NOTIFICATION("Setting camera render depth to %f %f\n", n, f);
        near = n; far = f; return 0;
    }

    /// Camera position and orientation setters
    int setCameraPosition(GLfloat x, GLfloat y, GLfloat z) {
        DEBUG_NOTIFICATION("Setting camera position to %f %f %f\n", x, y, z);
        pos_x = x; pos_y = y; pos_z = z; return 0;
    }
    int setCameraPositionDelta(GLfloat x, GLfloat y, GLfloat z) {
        DEBUG_NOTIFICATION("Setting camera position DELTA %f %f %f\n", x, y, z);
        pos_x += x; pos_y += y; pos_z += z; return 0;
    }
    int setCameraRotation(GLfloat x, GLfloat y, GLfloat z) {
        DEBUG_NOTIFICATION("Setting camera rotation to %f %f %f\n", x, y, z);
        rot_x = x; rot_y = y; rot_z = z; return 0;
    }
    int setCameraRotationDelta(GLfloat x, GLfloat y, GLfloat z) {
        DEBUG_NOTIFICATION("Setting camera position DELTA %f %f %f\n", x, y, z);
        rot_x += x; rot_y += y; rot_z += z; return 0;
    }

    /// Camera position and orientation getters
    int getCameraPosition(GLfloat *x, GLfloat *y, GLfloat *z) {
        DEBUG_NOTIFICATION("Getting camera position %f %f %f\n", pos_x, pos_y, pos_z);
        *x = pos_x; *y = pos_y; *z = pos_z; return 0;
    }
    int getCameraRotation(GLfloat *x, GLfloat *y, GLfloat *z) {
        DEBUG_NOTIFICATION("Getting camera rotation %f %f %f\n", rot_x, rot_y, rot_z);
        *x = rot_x; *y = rot_y; *z = rot_z; return 0;
    }

    int applyProjectionMatrix(void);
    int setCameraMode(CAMERA_MODE mode);
    int getID(void) { return ID; }

protected:

private:
    int ID;
    GLfloat pos_x, pos_y, pos_z;
    GLfloat rot_x, rot_y, rot_z;
    GLfloat lookat[4];
    GLfloat viewangle;
    GLfloat s_width, s_height;
    GLfloat near, far;
    CAMERA_MODE mode;
};

#endif // ScenegraphCamera_H
