
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "Scenegraph.h"
#include "RTexture.h"
#include "GLWrapper.h"
#include "DebugLog.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>

GLuint RTexture::loadETCTextureFromPKM(const char *filename)
{
    if (!Scenegraph::Instance()->queryFeature(SC_FEATURE_TEXTURING_ETC1))
    {
        DEBUG_INFO("ETC1 Texture loading not supported by this platform\n");
        DEBUG_INFO("Preprocessor flag GL_ETC1_RGB8_OES not defined\n");
        return 0;
    }
#if defined(GL_ETC1_RGB8_OES)
    unsigned int length;
    unsigned char *buffer;
    unsigned short w, h;
    GLuint texID;

    buffer = (unsigned char *) ResourceManager::Instance()->readBinaryFile(filename, &length);
    if (buffer == NULL)
    {
        return 0;
    }
    DEBUG_NOTIFICATION("PKM File read into memory, length %d bytes\n", (int)length);

    w = (buffer[8] << 8) + buffer[9];
    h = (buffer[10] << 8) + buffer[11];
    DEBUG_INFO("ETC1 image width %d and height %d\n", w, h);

    texID = 0;
    if (-1 == createCompressedGLTexture(&texID, &buffer[16], length-16, 0, w, h, GL_ETC1_RGB8_OES))
    {
        DEBUG_WARNING("Texture loading failed\n");
        return 0;
    }
    return texID;
#else
    return 0;
#endif
}
