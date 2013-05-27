
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef RTexture_H
#define RTexture_H

#include "Neocortex_GLHeaders.h"

#include <string>

#include "ResourceManager.h"
#include "GLWrapper.h"

// Class definition

class RTexture : public Resource
{
public:
    RTexture(int id);
    ~RTexture();

    // Mandatory API
    int bind(void);
    int destroy(void);
    int render(float deltatime);

    // Extra API, for texture resource type
    int fromFile(const char *filename);

private:
    // Methods
    GLuint loadETCTextureFromPKM(const char *filename);
    GLuint loadRGBTextureFromPNG(const char *filename);
    GLuint loadRGBTextureFromTGA(const char *filename);
    GLuint loadRGBTextureFromJPG(const char *filename);
    GLuint loadDXTTextureFromDDS(const char *filename);
    GLuint loadDXTTextureFromCRN(const char *filename);

    GLuint createGLTexture(unsigned char *buffer, GLuint width, GLuint height, bool hasAlpha);
    int createCompressedGLTexture(GLuint *textureID, unsigned char *buffer, GLuint level, GLuint length, GLuint width, GLuint height, GLuint format);

    //void queryCompressedTextureformats(void);

    // attributes
    GLuint texID;
};

class RTexture3D : public Resource
{
public:
    RTexture3D(int id);
    ~RTexture3D();

    // Mandatory API
    int bind(void);
    int destroy(void);
    int render(float deltatime);

    // Extra API, for texture resource type
    int fromFile(const char *filename);

private:
    // Methods
    GLuint loadETCTextureFromFile(const char *filename);
    GLuint loadRGBTexturefromPNG(const char *filename);

    // attributes
    GLuint texID;
};

#endif // RTexture_H
