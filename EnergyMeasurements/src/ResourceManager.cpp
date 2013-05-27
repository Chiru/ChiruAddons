
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "ResourceManager.h"
#include "RMesh.h"
#include "RTexture.h"
#include "RMaterial.h"

#include "GLWrapper.h"
#include "DebugLog.h"
#include "GLMath.h"
#include "UUID.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>

/******************************************************************************
 * Resource management singleton construction and destuction
 */

ResourceManager * ResourceManager::p_Instance = NULL;

ResourceManager * ResourceManager::Instance()
{
    if (p_Instance == NULL)
    {
        p_Instance = new ResourceManager();
    }
    return p_Instance;
}

ResourceManager::~ResourceManager()
{
    DEBUG_INFO("ResourceManager object destruction\n");
    for (std::map<int, Resource *>::iterator r=resources.begin(); r != resources.end(); r++)
    {
        delete (*r).second;
    }
    resources.clear();
    resources_named.clear();
}

/******************************************************************************
 * Resource management API
 */

int ResourceManager::createResource(RES_RESOURCE_TYPE resource_type, const char *filename)
{
    Resource *new_resource;

    // This test applies only to non-material types
    if (resource_type != RESOURCE_TYPE_MATERIAL)
    {
        new_resource = getResourceByName(sanitizeFilenameFromPath(filename));
        if (new_resource != NULL)
        {
            DEBUG_WARNING("Creating resource from file, but one with same name already exists\n");
            return new_resource->getID();
        }
    }

    switch (resource_type)
    {
    case RESOURCE_TYPE_MESH:
        RMesh *m;
        DEBUG_NOTIFICATION("Creating new mesh resource type %d, from file %s\n", resource_type, filename);
        m = new RMesh(UUID::Instance()->getUUID());
        if (-1 == m->setMeshFromFile(filename))
        {
            delete m;
            return -1;
        }
        new_resource = dynamic_cast<Resource *>(m);
        break;
    case RESOURCE_TYPE_TEXTURE:
        RTexture *t;
        DEBUG_NOTIFICATION("Creating new texture resource type %d, from file %s\n", resource_type, filename);
        t = new RTexture(UUID::Instance()->getUUID());
        if (-1 == t->fromFile(filename))
        {
            delete t;
            return -1;
        }
        new_resource = dynamic_cast<Resource *>(t);
        break;
    case RESOURCE_TYPE_TEXTURE_3D:
        RTexture *t3d;
        DEBUG_NOTIFICATION("Creating new 3D texture resource type %d, from file %s\n", resource_type, filename);
        t3d = new RTexture(UUID::Instance()->getUUID());
        if (-1 == t3d->fromFile(filename))
        {
            delete t3d;
            return -1;
        }
        new_resource = dynamic_cast<Resource *>(t3d);
        break;
    case RESOURCE_TYPE_MATERIAL:
        /*
         * Materials are special files, which can contain any number of material definitions.
         * Hence, it is not possible to create only one RMaterial, but instead parse the file
         * and create required number of materials. That is why material handling here
         * diffes from other resource types, which are immediate.
         */
        return parseMaterial(filename);
    default:
        DEBUG_INFO("Unknown resource type %d requested\n", resource_type);
        return -1;
    }

    DEBUG_INFO("New resource created succesfull, id = %d\n", new_resource->getID());
    new_resource->setType(resource_type);
    new_resource->setName(sanitizeFilenameFromPath(filename));
    resources.insert(std::pair<int, Resource *>(new_resource->getID(), new_resource));
    resources_named.insert(std::pair<const char *, Resource *>(new_resource->getName(), new_resource));
    return new_resource->getID();
}

const char *ResourceManager::sanitizeFilenameFromPath(const char *path)
{
    int i;
    for (i=strlen(path); i>=0; i--)
    {
        if (path[i] == '/' || path[i] == '\\')
            return &path[i+1];
    }
    return path;
}

int ResourceManager::deleteResource(int resource_id)
{
    Resource *r;

    r = getResource(resource_id);
    if (r == NULL) return -1;
    resources.erase(resource_id);
    resources_named.erase(r->getName());

    switch (r->getType())
    {
    case RESOURCE_TYPE_MESH:
        RMesh *r1;
        r1 = dynamic_cast <RMesh *> (r);
        delete r1;
        break;
    case RESOURCE_TYPE_TEXTURE:
    case RESOURCE_TYPE_TEXTURE_3D:
        RTexture *r2;
        r2 = dynamic_cast <RTexture *> (r);
        delete r2;
        break;
    case RESOURCE_TYPE_MATERIAL:
        RMaterial *mat;
        mat = dynamic_cast<RMaterial *> (r);
        delete mat;
        break;
    default:
        DebugLog::Instance()->MESSAGE(3, "Trying to delete a resource type %d which is not recognized\n", r->getType());
        break;
    }
    return 0;
}

int ResourceManager::deleteResourceByName(const char *n)
{
    Resource *r;
    r = getResourceByName(n);
    if (r == NULL) return -1;
    return deleteResource(r->getID());
}

Resource * ResourceManager::getResource(int resource_id)
{
    if (resources.find(resource_id) == resources.end())
    {
        DEBUG_NOTIFICATION("Trying find resource id=%d, which does not exist\n", resource_id);
        return NULL;
    }
    DEBUG_INFO("Resource query successful with id %d\n", resource_id);
    return resources[resource_id];
}

Resource * ResourceManager::getResourceByName(const char *n)
{
    for (std::map<const char *, Resource *>::iterator it=resources_named.begin(); it != resources_named.end(); ++it)
    {
        if (strcmp(it->first, n) == 0)
        {
            DEBUG_INFO("Resource query successful with name '%s'\n", n);
            return it->second;
        }
    }
    DEBUG_NOTIFICATION("Trying to find resource by name '%s', which does not exist\n", n);
    return NULL;
}

typedef enum {
    TOKEN_NONE = 0,
    TOKEN_MATERIAL,
    TOKEN_AMBIENT,
    TOKEN_DIFFUSE,
    TOKEN_SPECULAR,
    TOKEN_EMISSIVE,
    TOKEN_TEXTURE,
    TOKEN_TEXADDRESSMODE,
    TOKEN_PARAMNAMED,
    TOKEN_FILTERING,
    TOKEN_ALPHAREJECTION
} MAT_TOKEN;

typedef struct _mat_table {
    const char *name;
    MAT_TOKEN token;
} MATERIAL_LOOKUP;

MATERIAL_LOOKUP mat_table[] = {
    { "material", TOKEN_MATERIAL },
    { "ambient",  TOKEN_AMBIENT },
    { "diffuse",  TOKEN_DIFFUSE },
    { "specular", TOKEN_SPECULAR },
    { "emissive", TOKEN_EMISSIVE },
    { "texture",  TOKEN_TEXTURE },
    { "tex_addressing_mode", TOKEN_TEXADDRESSMODE },
    { "param_named", TOKEN_PARAMNAMED },
    { "filtering", TOKEN_FILTERING },
    { "alpha_rejection", TOKEN_ALPHAREJECTION }
};

char * ResourceManager::stripAndCompare(char *buf, const char *compare)
{
    char *ptr1, *ptr2;

    // First strip heading spaces and tabs
    ptr1 = buf;
    while (1)
    {
        if (*ptr1 == 0) return NULL;
        if (*ptr1 == ' ' || *ptr1 == '\t') ptr1++;
        else break;
    }

    // Seek next space, or end-of-line
    ptr2 = ptr1;
    while (1)
    {
        if (*ptr2 == ' ' || *ptr2 == '\n' || *ptr2 == 0) break;
        ptr2++;
    }
    *ptr2 = 0;
    ptr2++;

    // Compare
    if (strcmp(ptr1, compare) == 0)
    {
        return ptr2;
    }
    return NULL;
}

int ResourceManager::eraseTrailingNewline(char *s)
{
    unsigned int i;
    // first get rid of trailing newlines
    for (i=strlen(s)+1; i>0; i--)
    {
        if (s[i] == '\n' || s[i] == 0x0d) s[i] = 0;
    }
    // then spaces, if there are any
    for (i=strlen(s)-1; i>0; i--)
    {
        if (s[i] == ' ') s[i] = 0;
        else break;
    }
    return 0;
}

int ResourceManager::parseMaterial(const char *filename)
{
    FILE *file;
    char buffer[256];
    RMaterial *mat = NULL;
    unsigned int i, tex_channel = 0;
    float a, b, c;
    char buf1[64], buf2[64];
    bool skipmaterial = false;

    DEBUG_NOTIFICATION("Starting to parse material: %s\n", filename);

    file = fopen(filename, "r");
    if (file == NULL)
    {
        DEBUG_FATAL("Unable to open file %s\n", filename);
        return -1;
    }

    memset(buffer, 0, sizeof(buffer));
    while (fgets(buffer, sizeof(buffer), file))
    {
        //DEBUG_NOTIFICATION("Line read from material: %s ", buffer);
        for (i=0; i<sizeof(mat_table)/sizeof(MATERIAL_LOOKUP); i++)
        {
            char *params;
            params = stripAndCompare(buffer, mat_table[i].name);
            if (params != NULL)
            {
                DEBUG_INFO("Token '%s' found\n", mat_table[i].name);
                eraseTrailingNewline(params);

                // If next token is not a beginning of a new material, then do some sanitychecks
                // to avoid unnecessary parsing of the file.
                if (mat_table[i].token != TOKEN_MATERIAL)
                {
                    if (skipmaterial == true) continue;
                    if (mat == NULL)
                    {
                        DEBUG_WARNING("Unable to parse '%s' token, no material instance available\n", mat_table[i].name);
                        continue;
                    }
                }

                switch(mat_table[i].token)
                {
                case TOKEN_MATERIAL:
                    skipmaterial = false;
                    mat = dynamic_cast<RMaterial*>(getResourceByName(params));
                    if (mat != NULL)
                    {
                        DEBUG_WARNING("Trying to create material named '%s', but one already exists\n", params);
                        mat = NULL;
                        skipmaterial = true;
                        break;
                    }
                    DEBUG_NOTIFICATION("Starting new RMaterial() instance\n");
                    mat = new RMaterial(UUID::Instance()->getUUID());
                    mat->setName(params);
                    mat->setType(RESOURCE_TYPE_MATERIAL);
                    resources.insert(std::pair<int, Resource *>(mat->getID(), mat));
                    resources_named.insert(std::pair<const char *, Resource *>(mat->getName(), mat));
                    tex_channel = 0;
                    break;
                case TOKEN_AMBIENT:
                    sscanf(params, "%f %f %f", &a, &b, &c);
                    mat->setAmbient(a, b, c);
                    break;
                case TOKEN_DIFFUSE:
                    sscanf(params, "%f %f %f", &a, &b, &c);
                    mat->setDiffuse(a, b, c);
                    break;
                case TOKEN_EMISSIVE:
                    sscanf(params, "%f %f %f", &a, &b, &c);
                    mat->setEmissive(a, b, c);
                    break;
                case TOKEN_SPECULAR:
                    sscanf(params, "%f %f %f", &a, &b, &c);
                    mat->setSpecular(a, b, c);
                    break;
                case TOKEN_ALPHAREJECTION:
                    sscanf(params, "%s %f", (char *)&buf1, &a);
                    if (0 == strcmp("greater_equal", buf1)) mat->setAlphaRejection(MAT_AREJ_GREATEREQUAL, a);
                    break;
                case TOKEN_PARAMNAMED:
                    sscanf(params, "%s %s %f", (char *)&buf1, (char *)&buf2, &a);
                    DEBUG_NOTIFICATION("Scanning named material parameter %s, type %s, value %f\n", buf1, buf2, a);
                    if (0 == strcmp("specularPower", buf1))
                    {
                        mat->setSpecularPower(a);
                    }
                    else DEBUG_WARNING("Unrecognized named parameter\n");
                    break;
                case TOKEN_FILTERING:
                    sscanf(params, "%s", (char *)&buf1);
                    if (0 == strncmp(buf1, "none", strlen("none")))             mat->setTextureFilter(tex_channel, MAT_FILTER_NONE);
                    if (0 == strncmp(buf1, "bilinear", strlen("bilinear")))     mat->setTextureFilter(tex_channel, MAT_FILTER_BILINEAR);
                    if (0 == strncmp(buf1, "trilinear", strlen("trilinear")))   mat->setTextureFilter(tex_channel, MAT_FILTER_TRILINEAR);
                    if (0 == strncmp(buf1, "anistropic", strlen("anistropic"))) mat->setTextureFilter(tex_channel, MAT_FILTER_ANISOTROPIC);
                    break;
                case TOKEN_TEXTURE:
                    memset(buf1, 0, sizeof(buf1));
                    sscanf(params, "%s", &buf1[0]);
                    mat->setTextureName(tex_channel, (const char *)&buf1[0]);
                    tex_channel++;
                    break;
                default:
                    break;
                }
            }
        }
    }
    fclose(file);
    return 0;
}

/******************************************************************************
 * Utilities
 */

void * ResourceManager::readBinaryFile(const char *filename, unsigned int * length)
{
    struct stat results;
    unsigned char * buffer;

    if (stat(filename, &results) != 0)
    {
        DEBUG_NOTIFICATION("IOError when trying to access '%s'\n", filename);
        return NULL;
    }
    //results.st_size += 1; // Add one extra
    buffer = new unsigned char [results.st_size+1];
    if (length)
        *length = results.st_size;
    buffer[results.st_size] = 0; // Ensure trailing zero
    //memset(buffer, 0, results.st_size+1);

    DEBUG_NOTIFICATION("Trying to open '%s', length %d\n", filename, results.st_size);
    std::ifstream f(filename, std::ios::in | std::ios::binary);
    if (!f.read ((char *)buffer, results.st_size))
    {
        DEBUG_NOTIFICATION("IOERROR\n");
        return NULL;
    }
    f.close();
    return (void*) buffer;
}

/******************************************************************************
 * Private methods
 */
