/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "pugixml/pugixml.hpp"

#include "RMesh.h"
#include "RMeshContainer.h"

#include "GLWrapper.h"
#include "DebugLog.h"
#include "GLMath.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>

/// Constants, and local structures for parsing:

typedef enum {
    TOKEN_INVALID = 0,
    TOKEN_SHAREDGEOMETRY,
    TOKEN_SUBMESHES,
    TOKEN_VERTEXBUFFER,
    TOKEN_SUBMESH,
    TOKEN_VERTEX,
    TOKEN_POSITION,
    TOKEN_NORMAL,
    TOKEN_TEXCOORD,
    TOKEN_FACES,
    TOKEN_FACE,
    TOKEN_GEOMETRY,
    TOKEN_MESH
} _T;

typedef struct _token {
    const char *name;
    _T t;
} TOKEN;

static TOKEN table[] = {
    { "sharedgeometry", TOKEN_SHAREDGEOMETRY },
    { "submeshes",      TOKEN_SUBMESHES },
    { "vertexbuffer",   TOKEN_VERTEXBUFFER },
    { "submesh",        TOKEN_SUBMESH },
    { "vertex",         TOKEN_VERTEX },
    { "position",       TOKEN_POSITION },
    { "normal",         TOKEN_NORMAL },
    { "texcoord",       TOKEN_TEXCOORD },
    { "faces",          TOKEN_FACES  },
    { "face",           TOKEN_FACE },
    { "geometry",       TOKEN_GEOMETRY },
    { "mesh",           TOKEN_MESH }
};


/// De-serialisation state variables:

static int vertices = 0;
static int faces = 0;
static bool sg = false;
static bool hasSG = false;
static int submesh = -1;
static int cur_texcoord = 0;

static _T ogreXMLNameToToken(const char *name)
{
    unsigned int i;
    for (i=0; i<sizeof(table)/sizeof(TOKEN); i++)
    {
        if (strcmp(table[i].name, name) == 0)
        {
            return table[i].t;
        }
    }
    return TOKEN_INVALID;
}

static const char * getOgreXMLAttributeByName(pugi::xml_node n, const char *name)
{
    for (pugi::xml_attribute_iterator ait = n.attributes_begin(); ait != n.attributes_end(); ++ait)
    {
        if (strcmp(name, ait->name()) == 0)
        {
            return ait->value();
        }
    }
    return NULL;
}

RMeshContainer * RMesh::getCurrentMeshContainer(void)
{
    RMeshContainer *m = NULL;
    if (sg == true)
    {
        if (sharedgeometry == NULL)
        {
            DEBUG_INFO("Trying to add data into sharedgeometry, but sharedgeometry not allocated. Allocating now\n");
            sharedgeometry = new RMeshContainer;
        }
        m = sharedgeometry;
    }
    else
    {
        try
        {
            m = submeshes.at(submesh);
        }
        catch (std::exception &e)
        {
#ifdef WIN32
			e=e; // Supress compiler warning on Windows
#endif
            DEBUG_INFO("Creating new submesh %d for OgreXML\n", submesh);
            submeshes.resize(submesh+1, NULL);
            m = new RMeshContainer();
            submeshes.at(submesh) = m;
        }
    }
    return m;
}

static int parseOgreXMLNode(RMesh *parent, pugi::xml_node node)
{
    RMeshContainer *m;
    pugi::xml_node child;
    const char *t;

    //for (pugi::xml_node_iterator it = node.begin(); it != node.end(); ++it)
    //{
        switch (ogreXMLNameToToken(node.name()))
        {
        default:
        case TOKEN_INVALID:
            DEBUG_INFO("Unsupported OgreXML token %s\n", node.name());
            return 0;
        case TOKEN_MESH:
            // Reset state variables
            vertices = 0;
            faces = 0;
            sg = false;
            hasSG = false;
            submesh = -1;
            cur_texcoord = 0;
            break;
        case TOKEN_SHAREDGEOMETRY:
            t = getOgreXMLAttributeByName(node, "vertexcount");
            if (t == NULL)
            {
                DEBUG_CRITICAL("OgreXML sharedgeometry does not indicate vertexcount\n");
                return -1;
            }
            vertices = atoi(t);
            sg = true;
            hasSG = true;
            DEBUG_INFO("OgreXML sharedgeometry %d vertices\n", vertices);
            m = parent->getCurrentMeshContainer();
            m->reset();
            break;
        case TOKEN_VERTEXBUFFER:
            m = parent->getCurrentMeshContainer();
            //if (m == NULL) return -1;
            if (getOgreXMLAttributeByName(node, "positions") != NULL)
                if (-1 == m->allocAttribArray(ATTR_VERTEX, vertices, 0)) return -1;
            if (getOgreXMLAttributeByName(node, "normals") != NULL)
                if (-1 == m->allocAttribArray(ATTR_NORMAL, vertices, 0)) return -1;
            if (getOgreXMLAttributeByName(node, "colours") != NULL)
                if (-1 == m->allocAttribArray(ATTR_COLOR, vertices, 0)) return -1;
            t = getOgreXMLAttributeByName(node, "texture_coords");
            if (t != NULL)
            {
                int num = atoi(t);
                DEBUG_INFO("OgreXML mesh has %d parallel texture coordinate streams\n", num);
                if (num >= 1) m->allocAttribArray(ATTR_TEXCOORD_0, vertices, 0);
                if (num >= 2) m->allocAttribArray(ATTR_TEXCOORD_1, vertices, 0);
                if (num >= 3) m->allocAttribArray(ATTR_TEXCOORD_2, vertices, 0);
                if (num >= 4) m->allocAttribArray(ATTR_TEXCOORD_3, vertices, 0);
                if (num > 4)  DEBUG_INFO("OgreXML has more than 4 texture coordinate streams. Ignoring >= 4.");
            }
            break;
        case TOKEN_GEOMETRY:
            t = getOgreXMLAttributeByName(node, "vertexcount");
            if (t == NULL)
            {
                DEBUG_CRITICAL("OgreXML geometry does not indicate vertexcount\n");
                return -1;
            }
            vertices = atoi(t);
            DEBUG_INFO("OgreXML submesh geometry %d vertices\n", vertices);
            break;
        case TOKEN_POSITION:
            m = parent->getCurrentMeshContainer();
            //if (m == NULL) return -1;
            m->appendFloatAttribute(ATTR_VERTEX, (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "x")),
                                                 (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "y")),
                                                 (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "z")));
            break;
        case TOKEN_NORMAL:
            m = parent->getCurrentMeshContainer();
            //if (m == NULL) return -1;
            m->appendFloatAttribute(ATTR_NORMAL, (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "x")),
                                                 (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "y")),
                                                 (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "z")));
            break;
        case TOKEN_TEXCOORD:
            m = parent->getCurrentMeshContainer();
            //if (m == NULL) return -1;
            switch(cur_texcoord++)
            {
            case 0:
                m->appendFloatAttribute(ATTR_TEXCOORD_0, (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "u")),
                                                         (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "v")));
                break;
            case 1:
                m->appendFloatAttribute(ATTR_TEXCOORD_1, (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "u")),
                                                         (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "v")));
                break;
            case 2:
                m->appendFloatAttribute(ATTR_TEXCOORD_2, (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "u")),
                                                         (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "v")));
                break;
            case 3:
                m->appendFloatAttribute(ATTR_TEXCOORD_3, (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "u")),
                                                         (__NC_FLOAT_PRIMTYPE)atof(getOgreXMLAttributeByName(node, "v")));
                break;
            }
            break;
        case TOKEN_VERTEX:
            cur_texcoord = 0;
            break;
        case TOKEN_FACES:
            m = parent->getCurrentMeshContainer();
            t = getOgreXMLAttributeByName(node, "count");
            if (t == NULL)
            {
                DEBUG_CRITICAL("OgreXML faces do not indicate a face count\n");
                return -1;
            }
            faces = atoi(t);
            DEBUG_INFO("OgreXML submesh %d faces %d\n", submesh, faces);
            m->allocAttribArray(ATTR_INDEX, faces, vertices);
            break;
        case TOKEN_FACE:
            m = parent->getCurrentMeshContainer();
            //if (m == NULL) return -1;
            m->appendFixedAttribute(ATTR_INDEX, (__NC_FIXED_PRIMTYPE)atoi(getOgreXMLAttributeByName(node, "v1")),
                                                (__NC_FIXED_PRIMTYPE)atoi(getOgreXMLAttributeByName(node, "v2")),
                                                (__NC_FIXED_PRIMTYPE)atoi(getOgreXMLAttributeByName(node, "v3")));
            break;
        case TOKEN_SUBMESHES:
            sg = false;
            break;
        case TOKEN_SUBMESH:
            submesh++;
            // in case of sharedgeometry, we are not going to get new vertex count for each submesh.
            if (hasSG != true) vertices = 0;
            m = parent->getCurrentMeshContainer();
            t = getOgreXMLAttributeByName(node, "material");
            if (t != NULL)
            {
                m->setMaterial(t);
            }
            break;
        }

        // Parse into subtree, if any
        child = node.first_child();
        while (child)
        {
            if (-1 == parseOgreXMLNode(parent, child))
                return -1;
            child = child.next_sibling();
        }
//    }
    return 0;
}

static pugi::xml_node root;
int RMesh::parseOgreXMLRootNode(void)
{
    return parseOgreXMLNode(this, root);
}

int RMesh::setMeshFromOgreXML(const char *filename)
{
    int rc;
    pugi::xml_document doc;
    if (!doc.load_file(filename))
    {
        DEBUG_CRITICAL("OgreXML file '%s' open failed\n", filename);
        return -1;
    }

    // Ogre mesh starts with rootnode "mesh"
    root = doc.child("mesh");
    rc = parseOgreXMLRootNode();
    if (rc == 0)
    {
        DEBUG_INFO("OgreXML %s loaded successfully\n", filename)
    }
    return rc;
}
