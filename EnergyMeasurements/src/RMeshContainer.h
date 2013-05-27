
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef RMeshContainer_H
#define RMeshContainer_H

#include "Neocortex_GLHeaders.h"

#include <string>
#include <vector>
#include <stdlib.h>
#include <string.h>

#include "Neocortex_Datatypes.h"
#include "ResourceManager.h"
#include "GLWrapper.h"
#include "GLMath.h"

typedef enum {
    ATTR_VERTEX = 1,
    ATTR_NORMAL,
    ATTR_TEXCOORD_0,
    ATTR_TEXCOORD_1,
    ATTR_TEXCOORD_2,
    ATTR_TEXCOORD_3,
    ATTR_COLOR,
    ATTR_INDEX
} ATTR;


/// Class definition of RMeshContainer
class RMeshContainer {
public:
    RMeshContainer();
    ~RMeshContainer();

    int reset(void);
    int allocAttribArray(ATTR attrib, unsigned int len, unsigned int totalVertices);

    int appendFloatAttribute(ATTR attrib, __NC_FLOAT_PRIMTYPE a);
    int appendFloatAttribute(ATTR attrib, __NC_FLOAT_PRIMTYPE a, __NC_FLOAT_PRIMTYPE b);
    int appendFloatAttribute(ATTR attrib, __NC_FLOAT_PRIMTYPE a, __NC_FLOAT_PRIMTYPE b, __NC_FLOAT_PRIMTYPE c);
    int appendFloatAttribute(ATTR attrib, __NC_FLOAT_PRIMTYPE a, __NC_FLOAT_PRIMTYPE b, __NC_FLOAT_PRIMTYPE c, __NC_FLOAT_PRIMTYPE d);

    int appendFixedAttribute(ATTR attrib, __NC_FIXED_PRIMTYPE a);
    int appendFixedAttribute(ATTR attrib, __NC_FIXED_PRIMTYPE a, __NC_FIXED_PRIMTYPE b);
    int appendFixedAttribute(ATTR attrib, __NC_FIXED_PRIMTYPE a, __NC_FIXED_PRIMTYPE b, __NC_FIXED_PRIMTYPE c);
    int appendFixedAttribute(ATTR attrib, __NC_FIXED_PRIMTYPE a, __NC_FIXED_PRIMTYPE b, __NC_FIXED_PRIMTYPE c, __NC_FIXED_PRIMTYPE d);

    const char * getMaterial(void)   const { return &material[0]; }
    int setMaterial(const char *n) {
        DEBUG_NOTIFICATION("Setting Submesh material name to '%s'\n", n);
        memset(material, 0, sizeof(material));
        memcpy(material, n, std::min(sizeof(material)-1, strlen(n)));
        return 0;
    }

    int bindAttributeArrays(void);
    int render(float deltatime);

    /// ::finalize() is a method, which should be called after all the attributes are fed into the RMeshContainer.
    /// It makes sure the following issues:
    ///  - Index buffers are correct shape (smallest possible)
    ///  - attribute vectors are smallest shape (i.e. HALF_FLOATS)
    ///  - tangents are being created, if there is enough information in the mesh for it
    /// after finalize() the mesh should be considered locked, and not altered anymore. It can be done,
    /// but it should be noted that it will not be efficient.
    /// After this call, the mesh will be in the most efficient format to render for the runtime hardware.
    int finalize(RMeshContainer *sg);

private:
    int renderAsElementVBO(void);

    int createVBO(GLuint *VBO, unsigned int size, void * data, GLuint arraytype);
    int allocateAttributeVector(void **buffer, GLuint *gltype, unsigned int len, unsigned int elements, unsigned int *nItems, unsigned int *pItems);
    int appendAttribute(void **buffer, unsigned int *ptr, unsigned int type, GLfloat attr);

    int calculateTangents(RMeshContainer *sg);
    int optimizeIndexArray(RMeshContainer *sg);
    int optimizeAttribArray(void **b, unsigned int *len);

    GLushort floatToHalffloat(GLfloat *a);
    GLfloat halffloatToFloat(GLushort a);

    char material[128];

    /// Vectors holding the mesh data, pointers are void, since actual datatype is runtime defined
    void * vArray;
    void * nArray;
    void * cArray;
    void * t0Array;
    void * t1Array;
    void * t2Array;
    void * t3Array;
    void * iArray;
    GLuint iArray_type;
    GLuint vArray_type;

    /// Length of the above mentioned vectors
    unsigned int n_vArray;
    unsigned int n_nArray;
    unsigned int n_t0Array;
    unsigned int n_t1Array;
    unsigned int n_t2Array;
    unsigned int n_t3Array;
    unsigned int n_cArray;
    unsigned int n_iArray;

    /// Writepointers to the abovementioned vectors
    unsigned int p_vArray;
    unsigned int p_nArray;
    unsigned int p_t0Array;
    unsigned int p_t1Array;
    unsigned int p_t2Array;
    unsigned int p_t3Array;
    unsigned int p_cArray;
    unsigned int p_iArray;

    /// VBO for chosen geometry
    GLuint VBO_vertices;
    GLuint VBO_normals;
    GLuint VBO_texcoords0;
    GLuint VBO_texcoords1;
    GLuint VBO_texcoords2;
    GLuint VBO_texcoords3;
    GLuint VBO_colors;
    GLuint VBO_indices;

    /// Finalisation status of the mesh
    void *tangentArray;
    void *binormalArray;
};

#endif // RMeshContainer_H
