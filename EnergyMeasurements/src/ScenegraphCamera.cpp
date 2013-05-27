
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "ScenegraphCamera.h"
#include "GLMath.h"

ScenegraphCamera::ScenegraphCamera(int id) :
    pos_x(0.0f),
    pos_y(0.0f),
    pos_z(0.0f),
    rot_x(0.0f),
    rot_y(0.0f),
    rot_z(0.0f),
    viewangle(60.0f),
    s_width(800.0f),
    s_height(480.0f),
    near(1.0f),
    far(600.0f),
    mode(MODE_WASD)
{
    ID = id;
    lookat[0] = 0.0f;
    lookat[1] = 0.0f;
    lookat[2] = 1.0f;
    lookat[3] = 1.0f;
}

ScenegraphCamera::~ScenegraphCamera()
{
    DEBUG_INFO("Scenegraph camera %d destruction\n", ID);
}

/******************************************************************************
 * Camera methods
 */

int ScenegraphCamera::applyProjectionMatrix(void)
{
    DEBUG_NOTIFICATION("Using active camera %d for rendering\n", ID);
    GLMath::Instance()->_glMatrixMode(_GL_MATRIXMODE_PROJECTION);
    GLMath::Instance()->_glLoadIdentity();
    GLMath::Instance()->_glPerspective(viewangle, s_width/s_height, near, far);
    GLMath::Instance()->_glRotateEuler(rot_x, rot_y, rot_z);
    GLMath::Instance()->_glTranslate(-pos_x, -pos_y, -pos_z);
    return 0;
}


int ScenegraphCamera::setCameraMode(CAMERA_MODE m)
{
    mode = m;
    DEBUG_NOTIFICATION("Setting camera %d mode to %d\n", ID, mode);
    return 0;
}
