
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "RMesh.h"
#include "RShaderManager.h"

#include "ResourceManager.h"
#include "RTexture.h"
#include "RMaterial.h"

#include "GLWrapper.h"
#include "DebugLog.h"
#include "GLMath.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>

/******************************************************************************
 * Local constants for RMesh
 */

enum {
    // Filetypes:
    TYPE_UNKNOWN = 1,
    TYPE_OGREXML,
    TYPE_WAVEFRONT_OBJ
};

typedef struct _filetype {
    const char *ending;
    int type;
} FILETYPE;

static FILETYPE filetypes[] = {
    { "xml", TYPE_OGREXML },
    { "obj", TYPE_WAVEFRONT_OBJ }
};

/******************************************************************************
 * Construction and destruction
 */

RMesh::RMesh(int id) :
    sharedgeometry(NULL),
    pos_x(0.0f),
    pos_y(0.0f),
    pos_z(0.0f),
    rot_x(0.0f),
    rot_y(0.0f),
    rot_z(0.0f),
    sca_x(1.0f),
    sca_y(1.0f),
    sca_z(1.0f),
    finalized(false)
{
    ID = id;
    memset(textures, 0, sizeof(textures));
}

RMesh::~RMesh()
{
    DEBUG_INFO("Destruction of Rmesh, id %d, type %d\n", ID, type);
    destroy();
}

/******************************************************************************
 * File I/O
 */

int RMesh::getFileType(const char *filename)
{
    unsigned int l, i;

    l = strlen(filename);
    if (l<3)
    {
        DEBUG_WARNING("Invalid input filename '%s'\n", filename);
        return -1;
    }

    for (i=0; i<sizeof(filetypes)/sizeof(FILETYPE); i++)
    {
        if (filename[l-3]==filetypes[i].ending[0] &&
            filename[l-2]==filetypes[i].ending[1] &&
            filename[l-1]==filetypes[i].ending[2])
        {
            DEBUG_INFO("Fileformat %c%c%c recognized\n", filename[l-3], filename[l-2], filename[l-1]);
            return filetypes[i].type;
        }
    }
    return TYPE_UNKNOWN;
}

int RMesh::setMeshFromFile(const char *filename)
{
    int rc;
    switch(getFileType(filename))
    {
    case TYPE_OGREXML:
        rc = setMeshFromOgreXML(filename);
        break;
    case TYPE_WAVEFRONT_OBJ:
        rc = setMeshFromOBJ(filename);
        break;
    case TYPE_UNKNOWN:
    default:
        DEBUG_WARNING("Filetype not recognized\n");
        return -1;
    }
    if (rc == -1) return -1;
    return finalize();
}

int RMesh::setSubmeshTexture(int submesh, int channel, int texture_id)
{
    unsigned int i, j;
    if (submesh >= (int)submeshes.size())
    {
        DEBUG_FATAL("Trying to set a texture for submesh %d, and only %d submeshes available\n", submesh, submeshes.size());
        return -1;
    }
    if (channel >= MAX_TEX_CHANNELS)
    {
        DEBUG_FATAL("Trying to set a texture for channel %d, but only %d channels supported\n", channel, MAX_TEX_CHANNELS);
        return -1;
    }
    DEBUG_NOTIFICATION("Setting texture %d for submesh %d, tex channel %d\n", texture_id, submesh, channel);
    textures[submesh][channel] = texture_id;
    // Update occupied texture channel counts
    for (i=0; i<MAX_SUBMESHES; i++)
    {
        int count = 0;
        for (j=0; j<MAX_TEX_CHANNELS; j++)
        {
            if (textures[i][j] != 0)
                count++;
        }
        texture_count[i] = count;
    }
    return 0;
}

int RMesh::bind(void)
{
    return 0;
}

int RMesh::destroy(void)
{
    if (sharedgeometry != NULL)
        delete sharedgeometry;
    sharedgeometry = NULL;
    for (std::vector<RMeshContainer *>::iterator m=submeshes.begin(); m != submeshes.end(); m++)
    {
        delete (*m);
    }
    submeshes.clear();
    return 0;
}

int RMesh::render(float deltatime)
{
    return render(deltatime, NULL);
}

int RMesh::render(float deltatime, int *materials)
{
    unsigned int counter = 0;
    Resource *mat;

    if (sharedgeometry)
    {
        DEBUG_INFO("Rendering mesh with shared geometry %d\n", ID);
        sharedgeometry->bindAttributeArrays();
    }
    else DEBUG_INFO("Rendering mesh without shared geometry %d\n", ID);

    for (std::vector<RMeshContainer *>::iterator m=submeshes.begin(); m != submeshes.end(); m++)
    {
        DEBUG_INFO("Rendering submesh %d\n", counter);
        if (materials != NULL)
        {
            if (materials[counter] != -1)
            {
                DEBUG_INFO("Reading override material for the current mesh\n");
                mat = ResourceManager::Instance()->getResource(materials[counter]);
            }
            else
            {
                mat = ResourceManager::Instance()->getResourceByName((*m)->getMaterial());
            }
        }
        else
        {
            mat = ResourceManager::Instance()->getResourceByName((*m)->getMaterial());
        }
        if (mat == NULL)
        {
            DEBUG_WARNING("Material query %d for submesh %d failed. Skipping submesh render\n", (*m)->getMaterial(), counter);
            counter++;
            continue;
        }
        if (0 == mat->bind())
        {
            RShaderManager::Instance()->setTransformation(currentTransformation);
            if (!sharedgeometry) (*m)->bindAttributeArrays();
            (*m)->render(deltatime);
        }
        counter++;
    }
    DEBUG_INFO("Done rendering mesh\n");
    return 0;
}

int RMesh::applyTransformation(Matrix4X4 *m)
{
    DEBUG_NOTIFICATION("Setting mesh transformation\n");
    currentTransformation = m;
    return 0;
}

int RMesh::finalize(void)
{
    int i,j;
    // First collapse submeshesh with same material references

    i=0;
    j=0;
    for (std::vector<RMeshContainer*>::iterator it=submeshes.begin(); it != submeshes.end(); ++it)
    {
        j=i;
        for (std::vector<RMeshContainer*>::iterator it2=it+1; it2 != submeshes.end(); ++it2)
        {
            DEBUG_NOTIFICATION("comparing submesh %d to %d\n", i, j);
            if (0 == strcmp((*it)->getMaterial(), (*it2)->getMaterial()))
            {
                DEBUG_NOTIFICATION("Found submesh name match. Merging\n");
                //(*it)->merge((*it2));
                //delete (*it2);
            }
            else DEBUG_NOTIFICATION("No subname name match\n");
            j++;
        }
        //(*it)->finalize(sharedgeometry);
        i++;
    }


    for (std::vector<RMeshContainer*>::iterator it=submeshes.begin(); it != submeshes.end(); ++it)
    {
        (*it)->finalize(sharedgeometry);
    }
    return 0;
}
