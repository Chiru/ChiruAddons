
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef ResourceManager_H
#define ResourceManager_H

#include "Neocortex_GLHeaders.h"

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <string.h>
#include <stdlib.h>

#include "GLWrapper.h"
#include "UUID.h"
#include "DebugLog.h"

/// Resource types
typedef enum {
    RESOURCE_TYPE_MESH       = 1,
    RESOURCE_TYPE_TEXTURE    = 2,
    RESOURCE_TYPE_TEXTURE_3D = 3,
    RESOURCE_TYPE_MATERIAL   = 4
} RES_RESOURCE_TYPE;

/// Abstract class definition of Resource
class Resource
{
public:
    Resource()          { ID = UUID::Instance()->getUUID(); }
    Resource(int id)    { ID = id; }
    virtual ~Resource() {
        DEBUG_NOTIFICATION("Resource() Destruction\n");
    }

    // Getters for Resource()
    int getID()             const { return ID; }
    int getType()           const { return type; }
    const char * getName()  const { return (const char *)&name[0]; }

    // Setters for Resource()
    int setType(RES_RESOURCE_TYPE t) {
        DEBUG_NOTIFICATION("Setting resource type to %d\n", t);
        type = t;
        return 0;
    }
    int setName(const char *n) {
        DEBUG_NOTIFICATION("Setting resource name to %s\n", n);
        memset(name, 0, sizeof(name));
        memcpy(name, n, strlen(n));
        return 0;
    }

    virtual int bind(void) { return 0; }
    virtual int destroy(void) { return 0; }
    virtual int render(float deltatime) { return 0; }

protected:
    int ID;
    RES_RESOURCE_TYPE type;
    char name[64];
};

/// Class definition of ResourceManager
class ResourceManager
{
public:
    static ResourceManager * Instance();
    ~ResourceManager();

    /// Resource creation and destruction
    int createResource(RES_RESOURCE_TYPE resource_type, const char *filename);
    int deleteResource(int resource_id);
    int deleteResourceByName(const char *name);
    Resource * getResource(int resource_id);
    Resource * getResourceByName(const char *name);

    /// Utility functions
    void * readBinaryFile(const char *filename, unsigned int *length);

protected:

private:
    // Resource manager singleton
    ResourceManager() {}                                   // Private constructor
    ResourceManager(ResourceManager const &) {}
    static ResourceManager *p_Instance;                    // Single instance placeholder

    int setType(RES_RESOURCE_TYPE type);

    int parseMaterial(const char *filename);
    char * stripAndCompare(char *buf, const char *compare);
    int eraseTrailingNewline(char *s);
    const char * sanitizeFilenameFromPath(const char *path);

    std::map <int, Resource *> resources;
    std::map <const char *, Resource *> resources_named;
};

#endif // ResourceManager_H
