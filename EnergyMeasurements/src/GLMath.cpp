
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "GLMath.h"
#include "DebugLog.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include <assert.h>

#ifdef WIN32
#ifndef M_PI
#define M_PI 3.14159265f
#endif
#endif

/******************************************************************************
 * Global static instance of the class
 */

GLMath * GLMath::p_Instance = NULL;

GLMath * GLMath::Instance()
{
    if (p_Instance == NULL)
    {
        p_Instance = new GLMath();
    }
    return p_Instance;
}

GLMath::GLMath() :
    modelview_index(0),
    projection_index(0),
    matrixmode(_GL_MATRIXMODE_PROJECTION)
{
}

GLMath::~GLMath()
{
}

/******************************************************************************
 * Public GLMath methods
 */

Matrix4X4 * GLMath::getMatrix(MATRIXMODE mode)
{
    switch(mode)
    {
    case _GL_MATRIXMODE_PROJECTION:
        return &projection[projection_index];
    case _GL_MATRIXMODE_MODELVIEW:
        return &modelview[modelview_index];
    case _GL_MATRIXMODE_TRANSFORMATION:
        return &transformation;
    default:
        DEBUG_FATAL("GLMath, getMatrix(): invalid matrixmode\n");
        assert(0);
    }
    return NULL;
}

Matrix4X4 * GLMath::getTransformation(void)
{
    //_glMatrixMultiply(&transformation, getMatrix(_GL_MATRIXMODE_MODELVIEW), getMatrix(_GL_MATRIXMODE_PROJECTION));
    _glMatrixMultiply(&transformation, getMatrix(_GL_MATRIXMODE_PROJECTION), getMatrix(_GL_MATRIXMODE_MODELVIEW));
    return &transformation;
}

void GLMath::_glMatrixMode(MATRIXMODE mode)
{
    DEBUG_INFO("_glMatrixmode(%d)\n", mode);
    switch(mode)
    {
    case _GL_MATRIXMODE_PROJECTION:
    case _GL_MATRIXMODE_MODELVIEW:
        matrixmode = mode;
        return;
    default:
        DEBUG_NOTIFICATION("GLmath, _glMatrixmode(): invalid mode %d\n", mode);
    }
}

void GLMath::_glPushMatrix(void)
{
    switch (matrixmode)
    {
    case _GL_MATRIXMODE_PROJECTION:
        assert(projection_index < MATRIX_STACK_SIZE);
        memcpy(&projection[projection_index+1], &projection[projection_index], sizeof(Matrix4X4));
        projection_index++;
        DEBUG_NOTIFICATION("_glPushMatrix(), Projection matrix. New index %d\n", projection_index);
        break;
    case _GL_MATRIXMODE_MODELVIEW:
        assert(modelview_index < MATRIX_STACK_SIZE);
        memcpy(&modelview[modelview_index+1], &modelview[modelview_index], sizeof(Matrix4X4));
        modelview_index++;
        DEBUG_NOTIFICATION("_glPushMatrix(), Modelview matrix. New index %d\n", modelview_index);
        break;
    default:
        DEBUG_FATAL("GLMath, _glPushMatrix(): invalid matrixmode\n");
        assert(0);
    }
}

void GLMath::_glPopMatrix(void)
{
    switch(matrixmode)
    {
    case _GL_MATRIXMODE_PROJECTION:
        if (projection_index == 0)
        {
            DEBUG_WARNING("GLMath, _glPopMatrix(), tying to pop projection from empty stack.\n");
            break;
        }
        projection_index--;
        DEBUG_NOTIFICATION("_glPopMatrix(), Projection matrix. New index %d\n", projection_index);
        break;
    case _GL_MATRIXMODE_MODELVIEW:
        if (modelview_index == 0)
        {
            DEBUG_WARNING("GLMath, _glPopMatrix(), tying to pop modelview from empty stack.\n");
            break;
        }
        modelview_index--;
        DEBUG_NOTIFICATION("_glPopMatrix(), Modelview matrix. New index %d\n", modelview_index);
        break;
    default:
        DEBUG_FATAL("GLMath, _glPopMatrix(): invalid matrixmode\n");
        assert(0);
    }
}

void GLMath::_glLoadIdentity()
{
    Matrix4X4 *r;

    DEBUG_INFO("_glLoadIdentity()\n");
    r = getMatrix(matrixmode);

    _glLoadIdentity(r);
}

void GLMath::_glScale(GLfloat sx, GLfloat sy, GLfloat sz)
{
    Matrix4X4 *r;

    DEBUG_INFO("_glScale(%f, %f, %f)\n", sx, sy, sz);
    r = getMatrix(matrixmode);

    if (sx != 1.0f)
    {
        r->m[0][0] *= sx;
        r->m[0][1] *= sx;
        r->m[0][2] *= sx;
        r->m[0][3] *= sx;
    }

    if (sy != 1.0f)
    {
        r->m[1][0] *= sy;
        r->m[1][1] *= sy;
        r->m[1][2] *= sy;
        r->m[1][3] *= sy;
    }

    if (sz != 1.0f)
    {
        r->m[2][0] *= sz;
        r->m[2][1] *= sz;
        r->m[2][2] *= sz;
        r->m[2][3] *= sz;
    }
}

void GLMath::_glTranslate(GLfloat tx, GLfloat ty, GLfloat tz)
{
    Matrix4X4 *r;

    DEBUG_INFO("_glTranslate(%f, %f, %f)\n", tx, ty, tz);
    r = getMatrix(matrixmode);

    r->m[3][0] += (r->m[0][0]*tx + r->m[1][0]*ty + r->m[2][0]*tz);
    r->m[3][1] += (r->m[0][1]*tx + r->m[1][1]*ty + r->m[2][1]*tz);
    r->m[3][2] += (r->m[0][2]*tx + r->m[1][2]*ty + r->m[2][2]*tz);
    r->m[3][3] += (r->m[0][3]*tx + r->m[1][3]*ty + r->m[2][3]*tz);
}

void GLMath::_glRotate(GLfloat angle, GLfloat x, GLfloat y, GLfloat z)
{
   GLfloat sinAngle, cosAngle;
   GLfloat mag = sqrtf(x * x + y * y + z * z);
   Matrix4X4 *r;

   DEBUG_INFO("_glRotate(%f, %f, %f, %f)\n", angle, x, y, z);
   r = getMatrix(matrixmode);

   sinAngle = sinf( angle );//) * M_PI / 180.0f );
   cosAngle = cosf( angle );//* M_PI / 180.0f );
   if (mag > 0.0f)
   {
      GLfloat xx, yy, zz, xy, yz, zx, xs, ys, zs;
      GLfloat oneMinusCos;
      Matrix4X4 rotMat;

      x /= mag;
      y /= mag;
      z /= mag;

      xx = x * x;
      yy = y * y;
      zz = z * z;
      xy = x * y;
      yz = y * z;
      zx = z * x;
      xs = x * sinAngle;
      ys = y * sinAngle;
      zs = z * sinAngle;
      oneMinusCos = 1.0f - cosAngle;

      rotMat.m[0][0] = (oneMinusCos * xx) + cosAngle;
      rotMat.m[0][1] = (oneMinusCos * xy) - zs;
      rotMat.m[0][2] = (oneMinusCos * zx) + ys;
      rotMat.m[0][3] = 0.0f;

      rotMat.m[1][0] = (oneMinusCos * xy) + zs;
      rotMat.m[1][1] = (oneMinusCos * yy) + cosAngle;
      rotMat.m[1][2] = (oneMinusCos * yz) - xs;
      rotMat.m[1][3] = 0.0f;

      rotMat.m[2][0] = (oneMinusCos * zx) - ys;
      rotMat.m[2][1] = (oneMinusCos * yz) + xs;
      rotMat.m[2][2] = (oneMinusCos * zz) + cosAngle;
      rotMat.m[2][3] = 0.0f;

      rotMat.m[3][0] = 0.0f;
      rotMat.m[3][1] = 0.0f;
      rotMat.m[3][2] = 0.0f;
      rotMat.m[3][3] = 1.0f;

      _glMatrixMultiply(r, r, &rotMat);
   }
}

void GLMath::_glRotateEuler(GLfloat p, GLfloat h, GLfloat r)
{
    GLfloat cp, ch, cr;
    GLfloat sp, sh, sr;
    Matrix4X4 *m;
    Matrix4X4 rotmat;

    DEBUG_INFO("_glRotateEuler(%f, %f, %f)\n", p, h, r);
    m = getMatrix(matrixmode);

    cp = cos(p);
    cr = cos(r);
    ch = cos(h);
    sp = sin(p);
    sr = sin(r);
    sh = sin(h);

    DEBUG_NOTIFICATION("%f %f %f %f %f %f\n", cp, cr, ch, sp, sr, sh);

    memset(&rotmat, 0, sizeof(rotmat));

    rotmat.m[0][0] = cr*ch - sr*sp*sh;
    rotmat.m[1][0] = sr*ch + cr*sp*sh;
    rotmat.m[2][0] = -cp*sh;
    //rotmat.m[0][3] = 0.0f;

    rotmat.m[0][1] = -sr*cp;
    rotmat.m[1][1] = cr*cp;
    rotmat.m[2][1] = sp;
    //rotmat.m[1][3] = 0.0f;

    rotmat.m[0][2] = cr*sh + sr*sp*ch;
    rotmat.m[1][2] = sr*sh - cr*sp*ch;
    rotmat.m[2][2] = cp*ch;
    //rotmat.m[2][3] = 0.0f;

    //rotmat.m[3][0] = 0.0f;
    //rotmat.m[3][1] = 0.0f;
    //rotmat.m[3][2] = 0.0f;
    rotmat.m[3][3] = 1.0f;

    _glDebugMatrix(&rotmat);

    _glMatrixMultiply(m, m, &rotmat);
}

void GLMath::_glFrustum(float left, float right, float bottom, float top, float nearZ, float farZ)
{
    float       deltaX = right - left;
    float       deltaY = top - bottom;
    float       deltaZ = farZ - nearZ;
    Matrix4X4   *r;

    DEBUG_NOTIFICATION("glFrustum(%f, %f, %f, %f, %f, %f\n", left, right, bottom, top, nearZ, farZ);
    r = getMatrix(matrixmode);

    if ( (nearZ <= 0.0f) || (farZ <= 0.0f) ||
         (deltaX <= 0.0f) || (deltaY <= 0.0f) || (deltaZ <= 0.0f) )
         return;

    _glLoadIdentity(r);

    r->m[0][0] = 2.0f * nearZ / deltaX;
    //r->m[0][1] = r->m[0][2] = r->m[0][3] = 0.0f;

    r->m[1][1] = 2.0f * nearZ / deltaY;
    //r->m[1][0] = r->m[1][2] = r->m[1][3] = 0.0f;

    r->m[2][0] = (right + left) / deltaX;
    r->m[2][1] = (top + bottom) / deltaY;
    r->m[2][2] = -(nearZ + farZ) / deltaZ;
    r->m[2][3] = -1.0f;

    r->m[3][2] = -2.0f * nearZ * farZ / deltaZ;
    //r->m[3][0] = r->m[3][1] = r->m[3][3] = 0.0f;
    r->m[3][3] = 0.0f;

    //_glMatrixMultiply(r, r, &frust);
}


void GLMath::_glPerspective(float fovy, float aspect, float nearZ, float farZ)
{
   GLfloat frustumW, frustumH;

   DEBUG_INFO("_glPerspective(%f, %f, %f, %f)\n", fovy, aspect, nearZ, farZ);
   frustumH = tanf( fovy / 360.0f * M_PI ) * nearZ;
   frustumW = frustumH * aspect;

   _glFrustum(-frustumW, frustumW, -frustumH, frustumH, nearZ, farZ);
}

void GLMath::_glOrtho(float left, float right, float bottom, float top, float nearZ, float farZ)
{
    float       deltaX = right - left;
    float       deltaY = top - bottom;
    float       deltaZ = farZ - nearZ;
    Matrix4X4   *r;

    if ( (deltaX == 0.0f) || (deltaY == 0.0f) || (deltaZ == 0.0f) )
        return;

    r = getMatrix(matrixmode);

    _glLoadIdentity(r);
    r->m[0][0] = 2.0f / deltaX;
    r->m[3][0] = -(right + left) / deltaX;
    r->m[1][1] = 2.0f / deltaY;
    r->m[3][1] = -(top + bottom) / deltaY;
    r->m[2][2] = -2.0f / deltaZ;
    r->m[3][2] = -(nearZ + farZ) / deltaZ;

    //_glMatrixMultiply(result, &ortho, result);
}

void GLMath::MatMulVec4(Matrix4X4 *m, GLfloat *vec, GLfloat *res)
{
    for (unsigned int i=0; i<4; i++)
    {
        res[i] = m->m[0][i]*vec[0] + m->m[1][i]*vec[1] + m->m[2][i]*vec[2] + m->m[3][i]*vec[3];
    }
}

/******************************************************************************
 * Private GLMath methods
 */

void GLMath::_glMatrixMultiply(Matrix4X4 *result, Matrix4X4 *srcA, Matrix4X4 *srcB)
{
    Matrix4X4 tmp;
    int       i;

    DEBUG_NOTIFICATION("Matrix multiply A*B\n");
    //_glDebugMatrix(srcA);
    //_glDebugMatrix(srcB);

    for (i=0; i<4; i++)
    {
        tmp.m[0][i] = (srcA->m[0][i] * srcB->m[0][0]) +
                      (srcA->m[1][i] * srcB->m[0][1]) +
                      (srcA->m[2][i] * srcB->m[0][2]) +
                      (srcA->m[3][i] * srcB->m[0][3]);
        //DEBUG_NOTIFICATION("%f*%f + %f*%f + %f*%f + %f*%f = %f\n",
        //                   srcA->m[0][i], srcB->m[0][0], srcA->m[1][i], srcB->m[0][1],
        //                   srcA->m[2][i], srcB->m[0][2], srcA->m[3][i], srcB->m[0][3], tmp.m[0][i]);

        tmp.m[1][i] = (srcA->m[0][i] * srcB->m[1][0]) +
                      (srcA->m[1][i] * srcB->m[1][1]) +
                      (srcA->m[2][i] * srcB->m[1][2]) +
                      (srcA->m[3][i] * srcB->m[1][3]);

        tmp.m[2][i] = (srcA->m[0][i] * srcB->m[2][0]) +
                      (srcA->m[1][i] * srcB->m[2][1]) +
                      (srcA->m[2][i] * srcB->m[2][2]) +
                      (srcA->m[3][i] * srcB->m[2][3]);

        tmp.m[3][i] = (srcA->m[0][i] * srcB->m[3][0]) +
                      (srcA->m[1][i] * srcB->m[3][1]) +
                      (srcA->m[2][i] * srcB->m[3][2]) +
                      (srcA->m[3][i] * srcB->m[3][3]);
        //DEBUG_NOTIFICATION("%f %f %f %f\n", tmp.m[0][i], tmp.m[1][i], tmp.m[2][i], tmp.m[3][i]);
    }
    memcpy(result, &tmp, sizeof(Matrix4X4));
}

void GLMath::_glLoadIdentity(Matrix4X4 *r)
{
    memset(r, 0x0, sizeof(Matrix4X4));
    r->m[0][0] = 1.0f;
    r->m[1][1] = 1.0f;
    r->m[2][2] = 1.0f;
    r->m[3][3] = 1.0f;
}

void GLMath::_glDebugMatrix(MATRIXMODE mode)
{
#if defined(_DEBUG)
    Matrix4X4 *m;
    m = getMatrix(mode);
    _glDebugMatrix(m);
#endif
}

void GLMath::_glDebugMatrix(Matrix4X4 *mt)
{
#if defined(_DEBUG)
    float *m;
    m = (float*)mt;
    DEBUG_INFO(" [ Matrixmode = %d, addr 0x%x\n", matrixmode, m);
    for (int i=0; i<4; i++)
    {
        DEBUG_INFO(" %f %f %f %f\n", m[0], m[4], m[8], m[12]);
        //DEBUG_INFO(" %f %f %f %f\n", mt->m[0][i],mt->m[1][i],mt->m[2][i],mt->m[3][i]);
        m += 1;
    }
    DEBUG_INFO(" ]\n");
#endif
}
