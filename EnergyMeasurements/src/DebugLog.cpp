
/**
 * Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "Neocortex_Config.h"

#include "DebugLog.h"
#include <iostream>
#include <string.h>

/******************************************************************************
 * Global static instance of the class
 */

DebugLog * DebugLog::p_Instance = NULL;

DebugLog * DebugLog::Instance()
{
    if (p_Instance == NULL)
    {
        p_Instance = new DebugLog();
    }
    return p_Instance;
}

/******************************************************************************
 * Debug logging methods
 */

DebugLog::DebugLog() :
    verbosity(1)
{
    buffer = new char [10240]; // (char *) NC_MALLOC(10240, "DebugLog VA_ARGS buffer");
}

DebugLog::~ DebugLog()
{
    delete buffer; //NC_FREE(buffer);
}

void DebugLog::MESSAGE(int level, const char *format, ...)
{
    va_list args;
    va_start (args, format);
    vsprintf (buffer,format, args);
    outputMessage(level, buffer);
    va_end (args);
}

void DebugLog::outputMessage(int level, const char *message)
{
    if (verbosity >= level)
    {
        std::cout << level <<": " << message;
    }
}

void DebugLog::setVerbosityLevel(unsigned int level)
{
    verbosity = level;
}

/******************************************************************************
 * GfxDebugLog class
 */

#if defined(_NDEBUG)

GfxDebugLog * GfxDebugLog::p_Instance = NULL;

GfxDebugLog * GfxDebugLog::Instance()
{
    if (p_Instance == NULL)
    {
        p_Instance = new GfxDebugLog();
    }
    return p_Instance;
}

GfxDebugLog::GfxDebugLog() :
    batches(0),
    triangles(0),
    buffers(0),
    geometry(0),
    rgbtextures(0),
    comptextures(0),
    shaderbinding(0),
    texturebinding(0),
    frame(0),
    activetexdata(0)
{
    memset(texmap, -1, sizeof(texmap));
    memset(texsize, 0, sizeof(texsize));
    memset(frametexmap, -1, sizeof(frametexmap));
}

GfxDebugLog::~GfxDebugLog()
{
}

// Control API
void GfxDebugLog::resetCounters(void)
{
    Abatches[frame]        = batches;
    Atriangles[frame]      = triangles;
    Abuffers[frame]        = buffers;
    Ageometry[frame]       = geometry;
    Argbtextures[frame]    = rgbtextures;
    Acomptextures[frame]   = comptextures;
    Ashaderbinding[frame]  = shaderbinding;
    Atexturebinding[frame] = texturebinding;
    Aactivetexdata[frame]  = activetexdata;
    frame = (frame+1)%GFXDEBUG_MAX;

    batches         = 0;
    triangles       = 0;
    buffers         = 0;
    geometry        = 0;
    rgbtextures     = 0;
    comptextures    = 0;
    shaderbinding   = 0;
    texturebinding  = 0;
    activetexdata   = 0;
    memset(frametexmap, -1, sizeof(frametexmap));
    // Texture bindings
    // texture attr changes
    // shader bindings
}

void GfxDebugLog::printStatistics(void)
{
    unsigned int i;
    printf("GFX debug %d frames\n", frame);
    for (i=0; i<frame; i++)
    {
        printf("%d,%d,%d,%f,%d,%d,%d,%d,%d,%d,%d\n",
            i,Abatches[i],Abuffers[i],(float)Abuffers[i]/(float)Abatches[i],
            Atriangles[i],Ageometry[i],Argbtextures[i],Acomptextures[i],Ashaderbinding[i],
            Atexturebinding[i], Aactivetexdata[i]);
#if 0
        DEBUG_FATAL("GfxDebugLog statistics:\n");
        DEBUG_FATAL("  Rendered batches:              %6d\n", batches);
        DEBUG_FATAL("  Assigned bufferobjects:        %6d\n", buffers);
        DEBUG_FATAL("  Average BO / patch:              %4.1f\n", (float)buffers/(float)batches);
        DEBUG_INFO("  Rendered triangles:            %6d\n", triangles);
        DEBUG_INFO("  Geometry written to GPU bus:   %6d\n", geometry);
        DEBUG_INFO("  RGB Texture data submitted:    %6d\n", rgbtextures);
        DEBUG_INFO("  Compressed tex data submitted: %6d\n", comptextures);
        DEBUG_INFO("  Shader objects bound:          %6d\n", shaderbinding);
        DEBUG_INFO("  Texture objects bound:         %6d\n", texturebinding);
#endif
    }
}

// Data collection
void GfxDebugLog::addBatches(unsigned int b)
{
    batches += b;
}

void GfxDebugLog::addTriangles(unsigned int t)
{
    triangles += t;
}

void GfxDebugLog::addBuffers(unsigned int b)
{
    buffers += b;
}

void GfxDebugLog::addGeometryData(unsigned int d)
{
    geometry += d;
}

void GfxDebugLog::addRGBTextureData(unsigned int d)
{
    rgbtextures += d;
    for (unsigned int i=0; i<TEXDEBUG_MAX; i++)
    {
        if (texmap[i] == currenttexbind) break;
        if (texmap[i] == -1)
        {
            //printf("Caching tex id %d at index %d, size %d\n", currenttexbind, i, d);
            texmap[i] = currenttexbind;
            texsize[i] = d;
            activetexdata += d;
            break;
        }
    }
}

void GfxDebugLog::addCompTextureData(unsigned int d)
{
    comptextures += d;
}

void GfxDebugLog::addShaderBinding(void)
{
    shaderbinding += 1;
}

void GfxDebugLog::addTextureBinding(int id)
{
    texturebinding += 1;
    currenttexbind = id;
    //printf("bind tex %d\n", id);
    for (unsigned int i=0; i<TEXDEBUG_MAX; i++)
    {
        if (texmap[i] == id)
        {
            for (unsigned int j=0; j<TEXDEBUG_MAX; j++)
            {
                if (frametexmap[j] == id) return;
                if (frametexmap[j] == -1) {
                    //printf("Frame tex data fetch from index %d, id %d, %d bytes\n", i, id, texsize[i]);
                    activetexdata += texsize[i];
                    frametexmap[j] = id;
                    return;
                }
            }
            break;
        }
    }
}

#endif

