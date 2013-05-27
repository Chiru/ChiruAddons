/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "RTexture.h"
#include "GLWrapper.h"
#include "DebugLog.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>

#include "png.h"
#include "jpeglib.h"

RTexture::RTexture(int id) :
    texID(0)
{
    ID = id;
    //queryCompressedTextureformats();
}

RTexture::~RTexture()
{
    destroy();
}

/******************************************************************************
 * File I/O
 */

int RTexture::fromFile(const char *filename)
{
    unsigned int l;

    l = strlen(filename);
    if (filename[l-3]=='p' && filename[l-2]=='k' && filename[l-1]=='m')
    {
        DEBUG_INFO("Trying to open PKM/ETC1 texture file %s\n", filename);
        texID = loadETCTextureFromPKM(filename);
        if (texID == 0) return -1;
    }
    else if (filename[l-3]=='p' && filename[l-2]=='n' && filename[l-1]=='g')
    {
        DEBUG_INFO("Trying to open PNG texture file %s\n", filename);
        texID = loadRGBTextureFromPNG(filename);
        if (texID == 0) return -1;
    }
    else if (filename[l-3]=='t' && filename[l-2]=='g' && filename[l-1]=='a')
    {
        DEBUG_INFO("Trying to open TGA texture file %s\n", filename);
        texID = loadRGBTextureFromTGA(filename);
        if (texID == 0) return -1;
    }
    else if (filename[l-3]=='j' && filename[l-2]=='p' && filename[l-1]=='g')
    {
        DEBUG_INFO("Trying to open JPG texture file %s\n", filename);
        texID = loadRGBTextureFromJPG(filename);
        if (texID == 0) return -1;
    }
    else if (filename[l-3]=='d' && filename[l-2]=='d' && filename[l-1]=='s')
    {
        DEBUG_INFO("Trying to open DDS texture file %s\n", filename);
        texID = loadDXTTextureFromDDS(filename);
        if (texID == 0) return -1;
    }
    else if (filename[l-3]=='c' && filename[l-2]=='r' && filename[l-1]=='n')
    {
        DEBUG_INFO("Trying to open CRN texture file %s\n", filename);
        texID = loadDXTTextureFromCRN(filename);
        if (texID == 0) return -1;
    }
    else
    {
        DEBUG_CRITICAL("File %s not recognized. Abort!\n", filename);
        DEBUG_CRITICAL("%c %c %c\n", filename[l-3], filename[l-2], filename[l-1]);
        return -1;
    }
    return 0;
}

int RTexture::bind(void)
{
    if (texID != 0)
    {
        DEBUG_INFO("Binding texture %d\n", ID);
        GLWrapper::Instance()->GLBINDTEXTURE(GL_TEXTURE_2D, texID);
        return 0;
    }
    return -1;
}

int RTexture::destroy(void)
{
    if (texID != 0)
    {
        GLWrapper::Instance()->GLDELETETEXTURES(1, &texID);
        texID = 0;
        return 0;
    }
    return -1;
}

int RTexture::render(float deltatime)
{
    return 0;
}

GLuint RTexture::createGLTexture(unsigned char *buffer, GLuint width, GLuint height, bool hasAlpha)
{
    /*
     * GL texture generation part
     */
    GLuint textureID;

    GLWrapper::Instance()->GLGENTEXTURES(1, &textureID);
    GLWrapper::Instance()->GLBINDTEXTURE(GL_TEXTURE_2D, textureID);
    GLWrapper::Instance()->GLPIXELSTOREI(GL_UNPACK_ALIGNMENT, 1);

    GLWrapper::Instance()->GLTEXIMAGE2D(GL_TEXTURE_2D, 0, hasAlpha ? GL_RGBA : GL_RGB, width,
                 height, 0, hasAlpha ? GL_RGBA : GL_RGB, GL_UNSIGNED_BYTE,
                 buffer);

    // At this point we might have errors already in the pipe, and if so, we'll cancel
    if (GLWrapper::Instance()->GLGETERROR() != GL_NO_ERROR)
    {
        DEBUG_INFO("Texture loading aborted due to errors in glTexImage2D()\n");
        GLWrapper::Instance()->GLDELETETEXTURES(1, &textureID);
        return 0;
    }
    // Texture filtering defaults come from RMaterial class

    DEBUG_NOTIFICATION("Generating automatic mipmaps\n");
    //GLWrapper::Instance()->GLGENERATEMIPMAP(GL_TEXTURE_2D);

    return textureID;
}

int RTexture::createCompressedGLTexture(GLuint *textureID, unsigned char *buffer, GLuint length, GLuint level, GLuint width, GLuint height, GLuint format)
{
    // GL texture generation part
    if (*textureID == 0) GLWrapper::Instance()->GLGENTEXTURES(1, textureID);
    if (*textureID == 0) return -1;
    GLWrapper::Instance()->GLBINDTEXTURE(GL_TEXTURE_2D, *textureID);

    GLWrapper::Instance()->GLCOMPRESSEDTEXIMAGE2D(GL_TEXTURE_2D, level, format, width, height, 0, length, buffer);

    // This is critical. CompressedTexImage may return invalid ENUM and if so, we will cancel
    if (GLWrapper::Instance()->GLGETERROR() != GL_NO_ERROR)
    {
        DEBUG_INFO("Texture loading aborted due to errors in glCompressedTexImage2D()\n");
        GLWrapper::Instance()->GLDELETETEXTURES(1, textureID);
        return -1;
    }
    // Texture filtering defaults come from RMaterial class

    DEBUG_NOTIFICATION("Generating automatic mipmaps\n");
    //GLWrapper::Instance()->GLGENERATEMIPMAP(GL_TEXTURE_2D);

    return 0;
}
