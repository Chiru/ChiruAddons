
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "Scenegraph.h"
#include "RMeshContainer.h"

#include "GLWrapper.h"
#include "DebugLog.h"
#include "GLMath.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>
#include <math.h>

RMeshContainer::RMeshContainer() :
    vArray(NULL),
    nArray(NULL),
    cArray(NULL),
    t0Array(NULL),
    t1Array(NULL),
    t2Array(NULL),
    t3Array(NULL),
    iArray(NULL),
    n_vArray(0),
    n_nArray(0),
    n_t0Array(0),
    n_t1Array(0),
    n_t2Array(0),
    n_t3Array(0),
    n_cArray(0),
    n_iArray(0),
    p_vArray(0),
    p_nArray(0),
    p_t0Array(0),
    p_t1Array(0),
    p_t2Array(0),
    p_t3Array(0),
    p_cArray(0),
    p_iArray(0),
    VBO_vertices(0),
    VBO_normals(0),
    VBO_texcoords0(0),
    VBO_texcoords1(0),
    VBO_texcoords2(0),
    VBO_texcoords3(0),
    VBO_colors(0),
    VBO_indices(0),
    tangentArray(NULL),
    binormalArray(NULL)
{
    memset(material, 0, sizeof(material));
}

RMeshContainer::~RMeshContainer()
{
    DEBUG_NOTIFICATION("RMeshContainer() destruction\n");
    reset();
}

int RMeshContainer::allocateAttributeVector(void **buffer, GLuint *gltype, unsigned int len, unsigned int elements, unsigned int *nItems, unsigned int *pItems)
{
    if (*buffer != NULL) free(*buffer);
    if (len > 65535 && !Scenegraph::Instance()->queryFeature(SC_FEATURE_32BITINDICES))
    {
        DEBUG_WARNING("Attributes exceed 65535 and 32bit indices are not supported by the hardware\n");
        return -1;
    }

    DEBUG_INFO("Allocating attribute vector, elements %d, primsize=%d, allocsize %d bytes\n",
               len, sizeof(GLfloat), elements*len*sizeof(GLfloat));
    *buffer = (void *) malloc(elements*len*sizeof(GLfloat));
    *gltype = GL_FLOAT;
    if (*buffer == NULL) return -1;
    *nItems = elements*len;
    *pItems = 0;
    return 0;
}

/******************************************************************************
 * Geometry
 */

int RMeshContainer::allocAttribArray(ATTR attrib, unsigned int len, unsigned int totalVertices)
{
    switch(attrib)
    {
    case ATTR_VERTEX:
        if (0 != allocateAttributeVector(&vArray, &vArray_type, len, 3, &n_vArray, &p_vArray))
            return -1;
        break;
    case ATTR_NORMAL:
        return allocateAttributeVector(&nArray, &vArray_type, len, 3, &n_nArray, &p_nArray);
    case ATTR_COLOR:
        return allocateAttributeVector(&cArray, &vArray_type, len, 3, &n_cArray, &p_cArray);
    case ATTR_TEXCOORD_0:
        return allocateAttributeVector(&t0Array, &vArray_type, len, 2, &n_t0Array, &p_t0Array);
    case ATTR_TEXCOORD_1:
        return allocateAttributeVector(&t1Array, &vArray_type, len, 2, &n_t1Array, &p_t1Array);
    case ATTR_TEXCOORD_2:
        return allocateAttributeVector(&t2Array, &vArray_type, len, 2, &n_t2Array, &p_t2Array);
    case ATTR_TEXCOORD_3:
        return allocateAttributeVector(&t3Array, &vArray_type, len, 2, &n_t3Array, &p_t3Array);
    case ATTR_INDEX:
        DEBUG_INFO("Allocating preliminary (32-bit) buffer for indices, elements %d\n", len);
        if (iArray != NULL) free(iArray);
        DEBUG_INFO("Index vector elements=%d, primsize=%d, datalength=%d\n", 3*len, sizeof(GLuint), 3*len*sizeof(GLuint));
        iArray_type = GL_UNSIGNED_INT;
        iArray = (void *) malloc(3*len*sizeof(GLuint));
        if (iArray == NULL) return -1;
        n_iArray = 3*len;
        p_iArray = 0;
        break;
    }
    return 0;
}

int RMeshContainer::appendFloatAttribute(ATTR attrib, __NC_FLOAT_PRIMTYPE a)
{
    return appendFloatAttribute(attrib, a, 0.0f, 0.0f, 0.0f);
}

int RMeshContainer::appendFloatAttribute(ATTR attrib, __NC_FLOAT_PRIMTYPE a, __NC_FLOAT_PRIMTYPE b)
{
    return appendFloatAttribute(attrib, a, b, 0.0f, 0.0f);
}

int RMeshContainer::appendFloatAttribute(ATTR attrib, __NC_FLOAT_PRIMTYPE a, __NC_FLOAT_PRIMTYPE b, __NC_FLOAT_PRIMTYPE c)
{
    return appendFloatAttribute(attrib, a, b, c, 0.0f);
}

int RMeshContainer::appendAttribute(void **buffer, unsigned int *ptr, unsigned int type, GLfloat attr)
{
    if (*buffer == NULL)
    {
        DEBUG_CRITICAL("Trying to add attribute into buffer, which is not allocated!\n");
        return -1;
    }
    switch(type)
    {
    case GL_FLOAT:
        {
            GLfloat *p = (GLfloat *)*buffer;
            p[(*ptr)++] = (GLfloat)attr;
        }
        break;
    default:
        DEBUG_WARNING("Appending attribute to an array which is not GL_FLOAT. finalize() called too early?\n");
        return -1;
    }
    return 0;
}

int RMeshContainer::appendFloatAttribute(ATTR attrib, __NC_FLOAT_PRIMTYPE a, __NC_FLOAT_PRIMTYPE b, __NC_FLOAT_PRIMTYPE c, __NC_FLOAT_PRIMTYPE d)
{
    switch (attrib)
    {
    case ATTR_VERTEX:
        DEBUG_INFO("Adding vertex %f %f %f into vertex array\n", a, b, c);
        appendAttribute(&vArray, &p_vArray, vArray_type, a);
        appendAttribute(&vArray, &p_vArray, vArray_type, b);
        appendAttribute(&vArray, &p_vArray, vArray_type, c);
        break;
    case ATTR_COLOR:
        DEBUG_INFO("Adding color %f %f %f into color array\n", a, b, c);
        appendAttribute(&cArray, &p_cArray, vArray_type, a);
        appendAttribute(&cArray, &p_cArray, vArray_type, b);
        appendAttribute(&cArray, &p_cArray, vArray_type, c);
        break;
    case ATTR_NORMAL:
        DEBUG_INFO("Adding normal %f %f %f into normal array\n", a, b, c);
        appendAttribute(&nArray, &p_nArray, vArray_type, a);
        appendAttribute(&nArray, &p_nArray, vArray_type, b);
        appendAttribute(&nArray, &p_nArray, vArray_type, c);
        break;
    case ATTR_TEXCOORD_0:
        DEBUG_INFO("Adding texturecoord#0 %f %f into texcoord array\n", a, b);
        appendAttribute(&t0Array, &p_t0Array, vArray_type, a);
        appendAttribute(&t0Array, &p_t0Array, vArray_type, b);
        break;
    case ATTR_TEXCOORD_1:
        DEBUG_INFO("Adding texturecoord#1 %f %f into texcoord array\n", a, b);
        appendAttribute(&t1Array, &p_t1Array, vArray_type, a);
        appendAttribute(&t1Array, &p_t1Array, vArray_type, b);
        break;
    case ATTR_TEXCOORD_2:
        DEBUG_INFO("Adding texturecoord#2 %f %f into texcoord array\n", a, b);
        appendAttribute(&t2Array, &p_t2Array, vArray_type, a);
        appendAttribute(&t2Array, &p_t2Array, vArray_type, b);
        break;
    case ATTR_TEXCOORD_3:
        DEBUG_INFO("Adding texturecoord#3 %f %f into texcoord array\n", a, b);
        appendAttribute(&t3Array, &p_t3Array, vArray_type, a);
        appendAttribute(&t3Array, &p_t3Array, vArray_type, b);
        break;
    default:
    case ATTR_INDEX:
        DEBUG_CRITICAL("Trying to allocate float attributes (%d) which do not support float attribute data\n", attrib);
        return -1;
    }
    return 0;
}

int RMeshContainer::appendFixedAttribute(ATTR attrib, __NC_FIXED_PRIMTYPE a)
{
    return appendFixedAttribute(attrib, a, 0, 0, 0);
}

int RMeshContainer::appendFixedAttribute(ATTR attrib, __NC_FIXED_PRIMTYPE a, __NC_FIXED_PRIMTYPE b)
{
    return appendFixedAttribute(attrib, a, b, 0, 0);
}

int RMeshContainer::appendFixedAttribute(ATTR attrib, __NC_FIXED_PRIMTYPE a, __NC_FIXED_PRIMTYPE b, __NC_FIXED_PRIMTYPE c)
{
    return appendFixedAttribute(attrib, a, b, c, 0);
}

int RMeshContainer::appendFixedAttribute(ATTR attrib, __NC_FIXED_PRIMTYPE a, __NC_FIXED_PRIMTYPE b, __NC_FIXED_PRIMTYPE c, __NC_FIXED_PRIMTYPE d)
{
    switch(attrib)
    {
    case ATTR_INDEX:
        if (iArray == NULL)
        {
            DEBUG_CRITICAL("Trying to add indices into buffer, which is not allocated!\n");
            return -1;
        }
        DEBUG_INFO("Adding index %d %d %d into index array\n", a, b, c);
        switch(iArray_type)
        {
        case GL_UNSIGNED_INT:
            {
                GLuint *_iArray = (GLuint *) iArray;
                _iArray[p_iArray++] = (GLuint) a;
                _iArray[p_iArray++] = (GLuint) b;
                _iArray[p_iArray++] = (GLuint) c;
            }
            break;
        default:
            DEBUG_FATAL("Trying to add indices into index buffer, but datatype is not GL_UNSIGNED_INT. Did you finalize() too early?\n");
            return -1;
        }
        break;
    default:
        DEBUG_CRITICAL("Trying to allocate fixed attributes (%d) which do not support fixed attribute data\n", attrib);
        return -1;
    }
    return 0;
}

int RMeshContainer::reset(void)
{
    // Vertices
    if (vArray != NULL) free(vArray);
    vArray = NULL;
    n_vArray = 0;
    p_vArray = 0;
    // Normals
    if (nArray != NULL) free(nArray);
    nArray = NULL;
    n_nArray = 0;
    p_nArray = 0;
    // Colors
    if (cArray != NULL) free(cArray);
    cArray = NULL;
    n_cArray = 0;
    p_cArray = 0;
    // Texcoord 0
    if (t0Array != NULL) free(t0Array);
    t0Array = NULL;
    n_t0Array = 0;
    p_t0Array = 0;
    // Texcoord 1
    if (t1Array != NULL) free(t1Array);
    t1Array = NULL;
    n_t1Array = 0;
    p_t1Array = 0;
    // Texcoord 2
    if (t2Array != NULL) free(t2Array);
    t2Array = NULL;
    n_t2Array = 0;
    p_t2Array = 0;
    // Texcoord 3
    if (t3Array != NULL) free(t3Array);
    t3Array = NULL;
    n_t3Array = 0;
    p_t3Array = 0;
    // Indices
    if (iArray != NULL) free(iArray);
    iArray = NULL;
    n_iArray = 0;
    p_iArray = 0;
    if (tangentArray != NULL) free(tangentArray);
    tangentArray = NULL;
    if (binormalArray != NULL) free(binormalArray);
    binormalArray = NULL;
    return 0;
}

/******************************************************************************
 * Rendering
 */

int RMeshContainer::bindAttributeArrays(void)
{
    if (vArray_type == GL_FLOAT)
    {
        DEBUG_INFO("Binding GL_FLOAT attribute arrays\n");
        DEBUG_INFO("Trying to create VBO for vertices\n");
        createVBO(&VBO_vertices,   n_vArray*sizeof(GLfloat),  vArray, GL_ARRAY_BUFFER);
        DEBUG_INFO("Trying to create VBO for texcoords\n");
        createVBO(&VBO_texcoords0, n_t0Array*sizeof(GLfloat), t0Array, GL_ARRAY_BUFFER);
        DEBUG_INFO("Trying to create VBO for normals\n");
        createVBO(&VBO_normals,    n_nArray*sizeof(GLfloat),  nArray, GL_ARRAY_BUFFER);
        DEBUG_INFO("Trying to create VBO for colors\n");
        createVBO(&VBO_colors,     n_cArray*sizeof(GLfloat),  cArray, GL_ARRAY_BUFFER);
    }
    else // Half floats
    {
        DEBUG_INFO("Binding GL_HALF_FLOAT attribute arrays\n");
        DEBUG_INFO("Trying to create VBO for vertices\n");
        createVBO(&VBO_vertices,   n_vArray*sizeof(GLushort),  vArray, GL_ARRAY_BUFFER);
        DEBUG_INFO("Trying to create VBO for texcoords\n");
        createVBO(&VBO_texcoords0, n_t0Array*sizeof(GLushort), t0Array, GL_ARRAY_BUFFER);
        DEBUG_INFO("Trying to create VBO for normals\n");
        createVBO(&VBO_normals,    n_nArray*sizeof(GLushort),  nArray, GL_ARRAY_BUFFER);
        DEBUG_INFO("Trying to create VBO for colors\n");
        createVBO(&VBO_colors,     n_cArray*sizeof(GLushort),  cArray, GL_ARRAY_BUFFER);
    }

    // 0: for vertices
    DEBUG_INFO("Activating vertex data for shader program\n");
    GLWrapper::Instance()->GLBINDBUFFER(GL_ARRAY_BUFFER, VBO_vertices);
    GLWrapper::Instance()->GLVERTEXATTRIBPOINTER(0, 3, vArray_type, GL_FALSE, 0, NULL);
    GLWrapper::Instance()->GLENABLEVERTEXATTRIBARRAY(0);

    if (VBO_texcoords0)
    {
        // 1: for texcoords
        DEBUG_INFO("Activating texcoord data for shader program\n");
        GLWrapper::Instance()->GLBINDBUFFER(GL_ARRAY_BUFFER, VBO_texcoords0);
        GLWrapper::Instance()->GLVERTEXATTRIBPOINTER(1, 2, vArray_type, GL_FALSE, 0, NULL);
        GLWrapper::Instance()->GLENABLEVERTEXATTRIBARRAY(1);
    }
    else GLWrapper::Instance()->GLDISABLEVERTEXATTRIBARRAY(1);

    if (VBO_normals)
    {
        // 2: for normals
        DEBUG_INFO("Activating normal data for shader program\n");
        GLWrapper::Instance()->GLBINDBUFFER(GL_ARRAY_BUFFER, VBO_normals);
        GLWrapper::Instance()->GLVERTEXATTRIBPOINTER(2, 3, vArray_type, GL_FALSE, 0, NULL);
        GLWrapper::Instance()->GLENABLEVERTEXATTRIBARRAY(2);
    }
    else GLWrapper::Instance()->GLDISABLEVERTEXATTRIBARRAY(2);

    if (VBO_colors)
    {
        // 3: for colors
        DEBUG_INFO("Activating color data for shader program\n");
        GLWrapper::Instance()->GLBINDBUFFER(GL_ARRAY_BUFFER, VBO_colors);
        GLWrapper::Instance()->GLVERTEXATTRIBPOINTER(3, 3, vArray_type, GL_FALSE, 0, NULL);
        GLWrapper::Instance()->GLENABLEVERTEXATTRIBARRAY(3);
    }
    else GLWrapper::Instance()->GLDISABLEVERTEXATTRIBARRAY(3);

    return 0;
}

int RMeshContainer::render(float deltatime)
{
    return renderAsElementVBO();
}

int RMeshContainer::renderAsElementVBO(void)
{
    // Shader programs are formulated with following assumptions
    //
    // Attrib array ordering:
    // 0: for vertices      (3 element array)
    // 1: for texcoords     (2 element array)
    // 2: for normals       (3 element array)
    // 3: for colors        (3 element array)
    // and ELEMENT_ARRAY_BUFFER for indices

    DEBUG_INFO("Trying to create VBO for indices\n");
    switch(iArray_type)
    {
    case GL_UNSIGNED_BYTE:
        createVBO(&VBO_indices, n_iArray*sizeof(GLubyte),  iArray, GL_ELEMENT_ARRAY_BUFFER);
        break;
    case GL_UNSIGNED_SHORT:
        createVBO(&VBO_indices, n_iArray*sizeof(GLushort), iArray, GL_ELEMENT_ARRAY_BUFFER);
        break;
    case GL_UNSIGNED_INT:
        createVBO(&VBO_indices, n_iArray*sizeof(GLuint),   iArray, GL_ELEMENT_ARRAY_BUFFER);
        break;
    }

    // Fixme: RMeshContainer 486:
    // Fixme: 1) Make sure attribute array pointers are enough on current hardware
    // Fixme: 2) Rewrite container renderer to support multiple texturecoord channels
    // GL_ELEMENT_ARRAY_BUFFER for indices

    DEBUG_INFO("Activating index data for shader program\n");
    GLWrapper::Instance()->GLBINDBUFFER(GL_ELEMENT_ARRAY_BUFFER, VBO_indices);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);

#if 0
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
    GLWrapper::Instance()->GLDRAWELEMENTS(GL_TRIANGLES, n_iArray, iArray_type, NULL);
#endif
    // Generally a good idea to disable VBOs after rendering is done
    //DEBUG_INFO("Disabling VBO use\n");
    //GLWrapper::Instance()->GLBINDBUFFER(GL_ARRAY_BUFFER, 0);
    //GLWrapper::Instance()->GLBINDBUFFER(GL_ELEMENT_ARRAY_BUFFER, 0);
    return 0;
}

int RMeshContainer::createVBO(GLuint *VBO, unsigned int size, void * data, GLuint arraytype)
{
    if (*VBO == 0)      // If defined VBO is free
    {
        if (size == 0 || data == NULL)
        {
            DEBUG_NOTIFICATION("VBO creation skipped, data is NULL or len is zero\n");
            return 0;
        }
        GLWrapper::Instance()->GLGENBUFFERS(1, VBO);
        if (*VBO == 0)
        {
            DEBUG_NOTIFICATION("VBO creation failed\n");
            return -1;
        }
        GLWrapper::Instance()->GLBINDBUFFER(arraytype, *VBO);
        GLWrapper::Instance()->GLBUFFERDATA(arraytype, size, data, GL_STATIC_DRAW);
#if defined(_DEBUG)
        if (arraytype == GL_ARRAY_BUFFER)
        {
            if (Scenegraph::Instance()->queryFeature(SC_FEATURE_HALFFLOAT))
            {
                GLushort *d = (GLushort*) data;
                DEBUG_INFO("Data (first 4 elements): %f %f %f %f\n", halffloatToFloat(d[0]),
                                                                     halffloatToFloat(d[1]),
                                                                     halffloatToFloat(d[2]),
                                                                     halffloatToFloat(d[3]));
            }
            else
            {
                GLfloat *d = (GLfloat*) data;
                DEBUG_INFO("Data (first 4 elements): %f %f %f %f\n", d[0], d[1], d[2], d[3]);
            }
        }
        else
        {
            switch(iArray_type)
            {
            case GL_UNSIGNED_BYTE:
                 { GLubyte *d = (GLubyte *) data;
                   DEBUG_INFO("Data (first 4 elements): %d %d %d %d\n", d[0], d[1], d[2], d[3]); }
                break;
            case GL_UNSIGNED_SHORT:
                 { GLushort *d = (GLushort *) data;
                   DEBUG_INFO("Data (first 4 elements): %d %d %d %d\n", d[0], d[1], d[2], d[3]); }
                break;
            case GL_UNSIGNED_INT:
                 { GLuint *d = (GLuint *) data;
                   DEBUG_INFO("Data (first 4 elements): %d %d %d %d\n", d[0], d[1], d[2], d[3]); }
                break;
            }
        }
#endif
    }
#if defined(_DEBUG)
    else
    {
        DEBUG_WARNING("VBO already created\n");
    }
#endif
    return 0;
}

/******************************************************************************
 * float <-> halfFloat converters
 *
 * - Original reference code is from OpenGL ES 2.0 Programming Guide
 *   Aaftab Munshi, Dan Ginsburg, Dave Shreiner
 */

// -15 stored using a single precision bias of 127
const unsigned int  HALF_FLOAT_MIN_BIASED_EXP_AS_SINGLE_FP_EXP = 0x38000000;

// max exponent value in single precision that will be converted
// to Inf or Nan when stored as a half-float
const unsigned int  HALF_FLOAT_MAX_BIASED_EXP_AS_SINGLE_FP_EXP = 0x47800000;

// 255 is the max exponent biased value
const unsigned int  FLOAT_MAX_BIASED_EXP = (0xFF << 23);

const unsigned int  HALF_FLOAT_MAX_BIASED_EXP = (0x1F << 10);

GLushort RMeshContainer::floatToHalffloat(GLfloat *f)
{
    unsigned int    x = *(unsigned int *)f;
    unsigned int    sign = (unsigned short)(x >> 31);
    unsigned int    mantissa;
    unsigned int    exp;
    GLushort        hf;

    // get mantissa
    mantissa = x & ((1 << 23) - 1);
    // get exponent bits
    exp = x & FLOAT_MAX_BIASED_EXP;
    if (exp >= HALF_FLOAT_MAX_BIASED_EXP_AS_SINGLE_FP_EXP)
    {
        // check if the original single precision float number is a NaN
        if (mantissa && (exp == FLOAT_MAX_BIASED_EXP))
        {
            // we have a single precision NaN
            mantissa = (1 << 23) - 1;
        }
        else
        {
            // 16-bit half-float representation stores number as Inf
            mantissa = 0;
        }
        hf = (((GLushort)sign) << 15) | (GLushort)(HALF_FLOAT_MAX_BIASED_EXP) |
              (GLushort)(mantissa >> 13);
    }
    // check if exponent is <= -15
    else if (exp <= HALF_FLOAT_MIN_BIASED_EXP_AS_SINGLE_FP_EXP)
    {

        // store a denorm half-float value or zero
        exp = (HALF_FLOAT_MIN_BIASED_EXP_AS_SINGLE_FP_EXP - exp) >> 23;
        mantissa >>= (14 + exp);

        hf = (((GLushort)sign) << 15) | (GLushort)(mantissa);
    }
    else
    {
        hf = (((GLushort)sign) << 15) |
              (GLushort)((exp - HALF_FLOAT_MIN_BIASED_EXP_AS_SINGLE_FP_EXP) >> 13) |
              (GLushort)(mantissa >> 13);
    }
    return hf;
}

GLfloat RMeshContainer::halffloatToFloat(GLushort hf)
{
    unsigned int    sign = (unsigned int)(hf >> 15);
    unsigned int    mantissa = (unsigned int)(hf & ((1 << 10) - 1));
    unsigned int    exp = (unsigned int)(hf & HALF_FLOAT_MAX_BIASED_EXP);
    unsigned int    f;
    float *result;

    if (exp == HALF_FLOAT_MAX_BIASED_EXP)
    {
        // we have a half-float NaN or Inf
        // half-float NaNs will be converted to a single precision NaN
        // half-float Infs will be converted to a single precision Inf
        exp = FLOAT_MAX_BIASED_EXP;
        if (mantissa)
            mantissa = (1 << 23) - 1;    // set all bits to indicate a NaN
    }
    else if (exp == 0x0)
    {
        // convert half-float zero/denorm to single precision value
        if (mantissa)
        {
           mantissa <<= 1;
           exp = HALF_FLOAT_MIN_BIASED_EXP_AS_SINGLE_FP_EXP;
           // check for leading 1 in denorm mantissa
           while ((mantissa & (1 << 10)) == 0)
           {
               // for every leading 0, decrement single precision exponent by 1
               // and shift half-float mantissa value to the left
               mantissa <<= 1;
               exp -= (1 << 23);
            }
            // clamp the mantissa to 10-bits
            mantissa &= ((1 << 10) - 1);
            // shift left to generate single-precision mantissa of 23-bits
            mantissa <<= 13;
        }
    }
    else
    {
        // shift left to generate single-precision mantissa of 23-bits
        mantissa <<= 13;
        // generate single precision biased exponent value
        exp = (exp << 13) + HALF_FLOAT_MIN_BIASED_EXP_AS_SINGLE_FP_EXP;
    }

    f = (sign << 31) | exp | mantissa;
    result = (float *)&f;
    return *result;
}

int RMeshContainer::finalize(RMeshContainer *sg)
{
    DEBUG_NOTIFICATION("Starting RMeshContainer finalisation process()\n");

    // First, build tangents. This needs to be first in the list, since the calculations
    // are based on UINT indices, and FLOAT attributes.
    calculateTangents(sg);

    // first, check the sizes of the index buffers
    optimizeIndexArray(sg);

    // second, check the sizes of the attributes
    if (Scenegraph::Instance()->queryFeature(SC_FEATURE_HALFFLOAT))
    {
        if (sg)
        {
            if (sg->vArray_type == GL_FLOAT)
            {
                optimizeAttribArray(&sg->vArray,  &sg->n_vArray);
                optimizeAttribArray(&sg->nArray,  &sg->n_nArray);
                optimizeAttribArray(&sg->cArray,  &sg->n_cArray);
                optimizeAttribArray(&sg->t0Array, &sg->n_t0Array);
                optimizeAttribArray(&sg->t1Array, &sg->n_t1Array);
                optimizeAttribArray(&sg->t2Array, &sg->n_t2Array);
                optimizeAttribArray(&sg->t3Array, &sg->n_t3Array);
#if defined(NC_EGL)
                sg->vArray_type = GL_HALF_FLOAT_OES;
#elif defined(NC_GLX)
                sg->vArray_type = GL_HALF_FLOAT_ARB;
#endif
            }
        }
        else if (vArray_type == GL_FLOAT)
        {
            optimizeAttribArray(&vArray,  &n_vArray);
            optimizeAttribArray(&nArray,  &n_nArray);
            optimizeAttribArray(&cArray,  &n_cArray);
            optimizeAttribArray(&t0Array, &n_t0Array);
            optimizeAttribArray(&t1Array, &n_t1Array);
            optimizeAttribArray(&t2Array, &n_t2Array);
            optimizeAttribArray(&t3Array, &n_t3Array);
#if defined(NC_EGL)
            vArray_type = GL_HALF_FLOAT_OES;
#elif defined(NC_GLX)
            vArray_type = GL_HALF_FLOAT_ARB;
#endif
        }
    }

    DEBUG_NOTIFICATION("Done submesh finalisation\n");
    return 0;
}

int RMeshContainer::optimizeAttribArray(void **buf, unsigned int *len)
{
    if (*buf == NULL) return -1;
    if (*len == 0) return -1;

    GLushort *b = (GLushort *) malloc(*len*sizeof(GLushort));
    if (!b) return -1;
    GLfloat *data = (GLfloat *) *buf;

    DEBUG_NOTIFICATION("Transforming Attrib buffer to half floats, items %d\n", *len);
    for (unsigned int i=0; i<*len; i++)
    {
        b[i] = (GLushort) floatToHalffloat(&data[i]);
    }
    free (*buf);
    *buf = b;
    return 0;
}

/*
 * Derived from
 * Lengyel, Eric. Computing Tangent Space Basis Vectors for an Arbitrary Mesh. Terathon Software 3D Graphics Library, 2001.
 * http://www.terathon.com/code/tangent.html
 * noontz 2010
 */

int RMeshContainer::calculateTangents(RMeshContainer *sg)
{
    float *vertices;
    float *texcoords;
    float *normals;
    float *t_Array;
    float *bn_Array;
    unsigned int *indices;
    unsigned int n_vertices;

    float *tan1, *tan2;

    if (sg)
    {
        vertices = (float*)sg->vArray;
        texcoords = (float*)sg->t0Array;
        normals = (float *)sg->nArray;
        indices = (unsigned int *)iArray;
        n_vertices = sg->n_vArray;
        if (iArray_type != GL_UNSIGNED_INT || sg->vArray_type != GL_FLOAT)
        {
            DEBUG_WARNING("Index and vertex array types are not UINT/FLOAT for SG. Unable to calculate tangents\n");
            return -1;
        }
    }
    else
    {
        vertices = (float *)vArray;
        texcoords = (float *)t0Array;
        normals = (float *)nArray;
        indices = (unsigned int*)iArray;
        n_vertices = n_vArray;
        if (iArray_type != GL_UNSIGNED_INT || vArray_type != GL_FLOAT)
        {
            DEBUG_WARNING("Index and vertex array types are not UINT/FLOAT. Unable to calculate tangents\n");
            return -1;
        }
    }

    if (vertices == NULL || texcoords == NULL || indices == NULL || normals == NULL)
    {
        DEBUG_WARNING("Vertices, texcoords, normals or indices are not defined. Unable to calculate tangents\n");
        return -1;
    }

    if (sg)
    {
        if (sg->tangentArray == NULL)
        {
            DEBUG_NOTIFICATION("Tangent array not yet allocated for SG, allocating it for %d elements\n", n_vertices);
            sg->tangentArray = (void*)malloc(4*n_vertices/3*sizeof(GLfloat));
        }
        if (sg->binormalArray == NULL)
        {
            DEBUG_NOTIFICATION("Binormal array not yet allocated for SG, allocating it for %d elements\n", n_vertices);
            sg->binormalArray = (void*)malloc(n_vertices*sizeof(GLfloat));
        }
        t_Array = (float *)sg->tangentArray;
        bn_Array = (float *)sg->binormalArray;
    }
    else
    {
        if (tangentArray == NULL)
        {
            DEBUG_NOTIFICATION("Tangent array not yet allocated, allocating it for %d elements\n", n_vertices);
            tangentArray = (void*)malloc(4*n_vertices/3*sizeof(GLfloat));
        }
        if (binormalArray == NULL)
        {
            DEBUG_NOTIFICATION("Binormal array not yet allocated, allocating it for %d elements\n", n_vertices);
            binormalArray = (void*)malloc(n_vertices*sizeof(GLfloat));
        }
        t_Array = (float *)tangentArray;
        bn_Array = (float *)binormalArray;
    }

    if (t_Array == NULL || bn_Array == NULL)
    {
        DEBUG_WARNING("Allocation of tangent or binormal arrays failed. Unable to calculate tangents\n");
        return -1;
    }

    tan1 = (float *)malloc(n_vertices*sizeof(GLfloat));
    tan2 = (float *)malloc(n_vertices*sizeof(GLfloat));
    memset(tan1, 0, n_vertices*sizeof(GLfloat));
    memset(tan2, 0, n_vertices*sizeof(GLfloat));

    DEBUG_NOTIFICATION("Starting tangent calculation (n_iArray=%d)\n", n_iArray);
    for(unsigned int i=0; i<n_iArray/3; i++)
    {
        unsigned int i1, i2, i3;
        float *v1, *v2, *v3;
        float *w1, *w2, *w3;
        float x1, y1, z1, x2, y2, z2, s1, t1, s2, t2, r;

        i1 = indices[3*i+0];
        i2 = indices[3*i+1];
        i3 = indices[3*i+2];
        DEBUG_NOTIFICATION("Face indices %d %d %d\n", i1, i2, i3);
        v1 = &vertices[3*i1];
        v2 = &vertices[3*i2];
        v3 = &vertices[3*i3];
        DEBUG_NOTIFICATION("Face vertices (%1.2f,%1.2f,%1.2f) (%1.2f,%1.2f,%1.2f) (%1.2f,%1.2f,%1.2f)\n", v1[0], v1[1], v1[2], v2[0], v2[1], v2[2], v3[0], v3[1], v3[2]);
        w1 = &texcoords[2*i1];
        w2 = &texcoords[2*i2];
        w3 = &texcoords[2*i3];
        DEBUG_NOTIFICATION("Face texcoords (%1.2f,%1.2f) (%1.2f,%1.2f) (%1.2f,%1.2f)\n", w1[0], w1[1], w2[0], w2[1], w3[0], w3[1]);
        x1 = v2[0] - v1[0];
        x2 = v3[0] - v1[0];
        y1 = v2[1] - v1[1];
        y2 = v3[1] - v1[1];
        z1 = v2[2] - v1[2];
        z2 = v3[2] - v1[2];
        s1 = w2[0] - w1[0];
        s2 = w3[0] - w1[0];
        t1 = w2[1] - w1[1];
        t2 = w3[1] - w1[1];
        r = 1.0f / (s1*t2 - s2*t1);
        DEBUG_NOTIFICATION("Intermediate: (x1=%1.2f y1=%1.2f z1=%1.2f) (x2=%1.2f y2=%1.2f z2=%1.2f)\n", x1, y1, z1, x2, y2, z2);
        DEBUG_NOTIFICATION("Intermediate: s1=%1.2f t1=%1.2f s2=%1.2f t2=%1.2f -- r=%1.2f\n", s1, t1, s2, t2, r);

        v1 = &tan1[3*i1];
        v2 = &tan1[3*i2];
        v3 = &tan1[3*i3];
        v1[0] += ((t2 * x1 - t1 * x2) * r);
        v1[1] += ((t2 * y1 - t1 * y2) * r);
        v1[2] += ((t2 * z1 - t1 * z2) * r);
        v2[0] += ((t2 * x1 - t1 * x2) * r);
        v2[1] += ((t2 * y1 - t1 * y2) * r);
        v2[2] += ((t2 * z1 - t1 * z2) * r);
        v3[0] += ((t2 * x1 - t1 * x2) * r);
        v3[1] += ((t2 * y1 - t1 * y2) * r);
        v3[2] += ((t2 * z1 - t1 * z2) * r);
        DEBUG_NOTIFICATION("tan1 correction: (%1.2f,%1.2f,%1.2f) (%1.2f,%1.2f,%1.2f) (%1.2f,%1.2f,%1.2f)\n", v1[0], v1[1], v1[2], v2[0], v2[1], v2[2], v3[0], v3[1], v3[2]);

        v1 = &tan2[3*i1];
        v2 = &tan2[3*i2];
        v3 = &tan2[3*i3];
        v1[0] += ((s1 * x2 - s2 * x1) * r);
        v1[1] += ((s1 * y2 - s2 * y1) * r);
        v1[2] += ((s1 * z2 - s2 * z1) * r);
        v2[0] += ((s1 * x2 - s2 * x1) * r);
        v2[1] += ((s1 * y2 - s2 * y1) * r);
        v2[2] += ((s1 * z2 - s2 * z1) * r);
        v3[0] += ((s1 * x2 - s2 * x1) * r);
        v3[1] += ((s1 * y2 - s2 * y1) * r);
        v3[2] += ((s1 * z2 - s2 * z1) * r);
        DEBUG_NOTIFICATION("tan2 correction: (%1.2f,%1.2f,%1.2f) (%1.2f,%1.2f,%1.2f) (%1.2f,%1.2f,%1.2f)\n", v1[0], v1[1], v1[2], v2[0], v2[1], v2[2], v3[0], v3[1], v3[2]);
    }

    for (unsigned int i=0; i<n_vertices/3; i++)
    {
        float *n;
        float *t, *t2;
        float *tangents;
        float dot, len;
        float t_vec[3];
        float cross[3];

        n  = (float *)&normals[3*i]; // This is already normalized, right, otherwise will not work
        t  = (float *)&tan1[3*i];
        t2 = (float *)&tan2[3*i];
        tangents = (float *)&t_Array[4*i];

        // Gram-Schmidt ortogonalize:
        dot = n[0]*t[0] + n[1]*t[1] + n[2]*t[2]; // tangent projection length on the normal
        t_vec[0] = t[0] - dot*n[0];
        t_vec[1] = t[1] - dot*n[1];
        t_vec[2] = t[2] - dot*n[2];
        DEBUG_NOTIFICATION("Tangent dot product, should be zero (%f)\n", t_vec[0]*n[0]+t_vec[1]*n[1]+t_vec[2]*n[2]);

        len = sqrt(t_vec[0]*t_vec[0] + t_vec[1]*t_vec[1] + t_vec[2]*t_vec[2]);
        tangents[0] = t_vec[0] / len;
        tangents[1] = t_vec[1] / len;
        tangents[2] = t_vec[2] / len;

        // Handedness:
        cross[0] = n[1]*t_vec[2] - n[2]*t_vec[1];
        cross[1] = n[0]*t_vec[2] - n[2]*t_vec[0];
        cross[2] = n[0]*t_vec[1] - n[1]*t_vec[0];
        dot = cross[0]*t2[0] + cross[1]*t2[1] + cross[2]*t2[2];
        if (dot < 0.0f) tangents[3] = -1.0f;
        else tangents[3] = 1.0f;
    }

    free(tan1);
    free(tan2);
    return 0;
}

int RMeshContainer::optimizeIndexArray(RMeshContainer *sg)
{
    unsigned int vertices;
    if (sg) vertices = sg->n_vArray/3;
    else vertices = n_vArray/3;
    if (vertices <= 255 && iArray_type == GL_UNSIGNED_INT)
    {
        GLubyte *ptr;
        GLuint *data = (GLuint *) iArray;
        DEBUG_NOTIFICATION("Re-allocating index array (vertices=%d, indices=%d) as GL_UNSIGNED_BYTE\n", vertices, n_iArray);
        ptr = (GLubyte *) malloc(n_iArray*sizeof(GLubyte));
        if (ptr)
        {
            for (unsigned int i=0; i<n_iArray; i++)
                ptr[i] = (GLubyte) data[i];
            iArray_type = GL_UNSIGNED_BYTE;
            free(iArray);
            iArray = (void *) ptr;
        }
        else DEBUG_FATAL("Index buffer allocation failed\n");
    }
    else if (vertices <= 65535 && iArray_type == GL_UNSIGNED_INT)
    {
        GLushort *ptr;
        GLuint *data = (GLuint *) iArray;
        DEBUG_NOTIFICATION("Re-allocating index array (vertices=%d) as GL_UNSIGNED_SHORT\n", vertices);
        ptr = (GLushort *) malloc(n_iArray*sizeof(GLushort));
        if (ptr)
        {
            for (unsigned int i=0; i<n_iArray; i++)
                ptr[i] = (GLshort) data[i];
            iArray_type = GL_UNSIGNED_SHORT;
            free(iArray);
            iArray = (void *) ptr;
        }
        else DEBUG_FATAL("Index buffer allocation failed\n");
    }
    DEBUG_NOTIFICATION("Done reallocation\n");
    return 0;
}
