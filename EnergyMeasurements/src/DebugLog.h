
/**
 * Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef DebugLog_H
#define DebugLog_H

#include <stdarg.h>
#include <stdio.h>

class DebugLog {
public:
    static DebugLog * Instance();

    ~DebugLog();

    // Methods for messages & debug output
    void outputMessage(int level, const char *message);
    void setVerbosityLevel(unsigned int level);

    void MESSAGE(int level, const char *format, ...);

private:
    DebugLog();                                     // Private constructor
    DebugLog(DebugLog const &) {}
    static DebugLog *p_Instance;                    // Single instance placeholder

    int verbosity;
    char *buffer;
};

#if defined(_DEBUG)

#define DEBUG_INFO(a, ...) DebugLog::Instance()->MESSAGE(5, "INFO: %s():%d : " a, __FUNCTION__, __LINE__, ## __VA_ARGS__);
#define DEBUG_NOTIFICATION(a, ...) DebugLog::Instance()->MESSAGE(4, "NOTE: %s():%d : " a, __FUNCTION__, __LINE__, ## __VA_ARGS__);
#define DEBUG_WARNING(a, ...) DebugLog::Instance()->MESSAGE(3, "WARNING: %s():%d : " a, __FUNCTION__, __LINE__, ## __VA_ARGS__);
#define DEBUG_CRITICAL(a, ...) DebugLog::Instance()->MESSAGE(2, "CRITICAL: %s():%d : " a, __FUNCTION__, __LINE__, ## __VA_ARGS__);
#define DEBUG_FATAL(a, ...) DebugLog::Instance()->MESSAGE(1, "FATAL: %s():%d : " a, __FUNCTION__, __LINE__, ## __VA_ARGS__);

#else // defined(_DEBUG)

#define DEBUG_INFO(a, ...)
#define DEBUG_NOTIFICATION(a, ...)
#define DEBUG_WARNING(a, ...)
#define DEBUG_CRITICAL(a, ...) DebugLog::Instance()->MESSAGE(2, "CRITICAL: %s():%d : " a, __FUNCTION__, __LINE__, ## __VA_ARGS__);
#define DEBUG_FATAL(a, ...) DebugLog::Instance()->MESSAGE(1, "FATAL: %s():%d : " a, __FUNCTION__, __LINE__, ## __VA_ARGS__);

#endif

/******************************************************************************
 * GfxDebugLog class collects statistics
 */


#define GFXDEBUG_MAX 5000
#define TEXDEBUG_MAX 250

#if defined(_NDEBUG)

class GfxDebugLog {
public:
    static GfxDebugLog * Instance();
    ~GfxDebugLog();

    // Control API
    void resetCounters(void);
    void printStatistics(void);

    // Data collection
    void addBatches(unsigned int b);
    void addTriangles(unsigned int t);
    void addBuffers(unsigned int b);
    void addGeometryData(unsigned int b);
    void addRGBTextureData(unsigned int b);
    void addCompTextureData(unsigned int b);
    void addShaderBinding(void);
    void addTextureBinding(int id);

private:
    GfxDebugLog();                                     // Private constructor
    GfxDebugLog(GfxDebugLog const &) {};
    static GfxDebugLog *p_Instance;                    // Single instance placeholder

    unsigned int batches;
    unsigned int triangles;
    unsigned int buffers;
    unsigned int geometry;
    unsigned int rgbtextures;
    unsigned int comptextures;
    unsigned int shaderbinding;
    unsigned int texturebinding;

    unsigned int Abatches[GFXDEBUG_MAX];
    unsigned int Atriangles[GFXDEBUG_MAX];
    unsigned int Abuffers[GFXDEBUG_MAX];
    unsigned int Ageometry[GFXDEBUG_MAX];
    unsigned int Argbtextures[GFXDEBUG_MAX];
    unsigned int Acomptextures[GFXDEBUG_MAX];
    unsigned int Ashaderbinding[GFXDEBUG_MAX];
    unsigned int Atexturebinding[GFXDEBUG_MAX];
    unsigned int Aactivetexdata[GFXDEBUG_MAX];
    unsigned int frame;
    int currenttexbind;
    int activetexdata;
    int texmap[TEXDEBUG_MAX];
    int texsize[TEXDEBUG_MAX];
    int frametexmap[TEXDEBUG_MAX];
};


#define GFXDEBUG_RESET()                GfxDebugLog::Instance()->resetCounters();
#define GFXDEBUG_PRINTSTATS()           GfxDebugLog::Instance()->printStatistics();
#define GFXDEBUG_ADDBATCHES(a)          GfxDebugLog::Instance()->addBatches(a);
#define GFXDEBUG_ADDTRIANGES(a)         GfxDebugLog::Instance()->addTriangles(a);
#define GFXDEBUG_ADDBUFFERS(a)          GfxDebugLog::Instance()->addBuffers(a);
#define GFXDEBUG_ADDGEOMETRYDATA(a)     GfxDebugLog::Instance()->addGeometryData(a);
#define GFXDEBUG_ADDRGBTEXTUREDATA(a)   GfxDebugLog::Instance()->addRGBTextureData(a);
#define GFXDEBUG_ADDCOMPTEXTUREDATA(a)  GfxDebugLog::Instance()->addCompTextureData(a);
#define GFXDEBUG_SHADERBINDING()        GfxDebugLog::Instance()->addShaderBinding();
#define GFXDEBUG_TEXTUREBINDING(d)      GfxDebugLog::Instance()->addTextureBinding(d);

#else

#define GFXDEBUG_RESET()
#define GFXDEBUG_PRINTSTATS()
#define GFXDEBUG_ADDBATCHES(a)
#define GFXDEBUG_ADDTRIANGES(a)
#define GFXDEBUG_ADDBUFFERS(a)
#define GFXDEBUG_ADDGEOMETRYDATA(a)
#define GFXDEBUG_ADDRGBTEXTUREDATA(a)
#define GFXDEBUG_ADDCOMPTEXTUREDATA(a)
#define GFXDEBUG_SHADERBINDING()
#define GFXDEBUG_TEXTUREBINDING(d)

#endif

#endif // DebugLog_H
