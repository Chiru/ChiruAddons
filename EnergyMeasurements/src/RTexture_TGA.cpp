
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

/*
 * loadRGBtextureFromTGA. A simple TGA file loader, which does not support compressed TGA files,
 * not it does support TGA's with alpha channel. Yet.
 */

GLuint RTexture::loadRGBTextureFromTGA(const char *filename)
{
    unsigned char *buffer = NULL;
    FILE *f;
    unsigned char tgaheader[12];
    unsigned char attributes[6];
    unsigned int imagesize;
    unsigned int width, height;
    GLuint texID;

    f = fopen(filename, "rb");
    if (f == NULL)
    {
        DEBUG_FATAL("TGA file %s open failed\n", filename);
        return 0;
    }

    if(fread(&tgaheader, sizeof(tgaheader), 1, f) == 0)
    {
        fclose(f);
        return 0;
    }

    if(fread(attributes, sizeof(attributes), 1, f) == 0)
    {
        fclose(f);
        return 0;
    }

    width = attributes[1] * 256 + attributes[0];
    height = attributes[3] * 256 + attributes[2];
    imagesize = attributes[4] / 8 * width * height;
    DEBUG_INFO("TGA file width %d, height %d and image size %d bytes\n", width, height, imagesize);

    buffer = new unsigned char[imagesize];
    if (buffer == NULL)
    {
        fclose(f);
        return 0;
    }

    if(fread(buffer, 1, imagesize, f) != imagesize)
    {
        delete buffer; //free(buffer);
        fclose(f);
        return 0;
    }
    fclose(f);

    /*
     * GL texture generation part
     */
    texID = createGLTexture(buffer, width, height, false);
    if (texID == 0) delete buffer;
    return texID;
}
