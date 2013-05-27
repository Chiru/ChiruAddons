
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef GLMath_H
#define GLMath_H

#include "Neocortex_GLHeaders.h"

typedef struct
{
    GLfloat  m[4][4];
} Matrix4X4;

typedef unsigned short MATRIXINDEX;

typedef enum {
    _GL_MATRIXMODE_PROJECTION = 1,
    _GL_MATRIXMODE_MODELVIEW = 2,
    _GL_MATRIXMODE_TRANSFORMATION = 3
} MATRIXMODE;

#define MATRIX_STACK_SIZE (16)


class GLMath {
public:
    static GLMath * Instance();
    ~GLMath();

    Matrix4X4 * getMatrix(MATRIXMODE mode);
    Matrix4X4 * getTransformation(void);
    void _glDebugMatrix(MATRIXMODE mode);
    void _glDebugMatrix(Matrix4X4 *mt);

    void _glMatrixMode(MATRIXMODE mode);
    void _glPushMatrix(void);
    void _glPopMatrix(void);
    void _glLoadIdentity();

    void _glScale(GLfloat sx, GLfloat sy, GLfloat sz);
    void _glTranslate(GLfloat tx, GLfloat ty, GLfloat tz);
    void _glRotate(GLfloat angle, GLfloat x, GLfloat y, GLfloat z);
    void _glRotateEuler(GLfloat p, GLfloat h, GLfloat r);

    void _glFrustum(float left, float right, float bottom, float top, float nearZ, float farZ);
    void _glPerspective(float fovy, float aspect, float nearZ, float farZ);
    void _glOrtho(float left, float right, float bottom, float top, float nearZ, float farZ);

    void MatMulVec4(Matrix4X4 *m, GLfloat *vec, GLfloat *res);

private:
    GLMath();                                     // Private constructor
    GLMath(GLMath const &) {}
    static GLMath *p_Instance;                    // Single instance placeholder

    void _glMatrixMultiply(Matrix4X4 *result, Matrix4X4 *srcA, Matrix4X4 *srcB);
    void _glLoadIdentity(Matrix4X4 *result);

    Matrix4X4 modelview[MATRIX_STACK_SIZE];
    Matrix4X4 projection[MATRIX_STACK_SIZE];
    MATRIXINDEX modelview_index;
    MATRIXINDEX projection_index;

    Matrix4X4 transformation;

    MATRIXMODE matrixmode;
};

#endif // GLMath_H
