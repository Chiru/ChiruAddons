
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "RShaderManager.h"
#include "ResourceManager.h"
#include "RShader.h"
#include "RShader_Shaders.h"

#include "DebugLog.h"

static SHADERLOOKUPTABLE shadertable[] = {
    { SHADER_SIMPLE, Simple_vertex, Simple_fragment },
    { SHADER_TEXTUREMAP_1CHANNEL, Texturemap_1Channel_vertex, Texturemap_1Channel_fragment },
    { SHADER_TEXTUREMAP_2CHANNEL_LIGHTMAP, Texturemap_2Channel_Lightmap_vertex, Texturemap_2Channel_Lightmap_fragment },
    { SHADER_TEXTUREMAP_3CHANNEL, Texturemap_3Channel_vertex, Texturemap_3Channel_fragment },
    { SHADER_TEXTUREMAP_4CHANNEL, Texturemap_4Channel_vertex, Texturemap_4Channel_fragment }
};


/******************************************************************************
 * Construction and destruction
 */

RShaderManager * RShaderManager::p_Instance = NULL;

RShaderManager * RShaderManager::Instance()
{
    if (p_Instance == NULL)
    {
        p_Instance = new RShaderManager();
    }
    return p_Instance;
}

RShaderManager::~RShaderManager()
{
    flush();
}

/******************************************************************************
 * I/O
 */

int RShaderManager::bind(SHADER_TYPE type)
{
    RShader * s;
    if (shaderCache.find(type) == shaderCache.end())
    {
        unsigned int i;
        int found = -1;
        for (i=0; i<sizeof(shadertable)/sizeof(SHADERLOOKUPTABLE); i++)
        {
            if (shadertable[i].type == type)
            {
                found = i;
                break;
            }
        }
        if (found == -1)
        {
            DEBUG_FATAL("Trying to bind shader %d, which does not exist in the shader lookup table\n", type);
            return -1;
        }
        s = new RShader;
        if (-1 == s->fromVectors((char*)shadertable[i].vertexshader, (char*)shadertable[i].fragmentshader))
        {
            delete s;
            return -1;
        }
        shaderCache.insert(std::pair<SHADER_TYPE, RShader *>(type, s));
        currentProgramObject = s->getProgramObject();
        return 0;
    }
    DEBUG_INFO("Binding a shader from cache for type %d\n", type);
    s = shaderCache[type];
    if (currentProgramObject == s->getProgramObject())
    {
        DEBUG_NOTIFICATION("Shader object %d already bound. Skipping.\n", currentProgramObject);
        return 0;
    }
    currentProgramObject = s->getProgramObject();
    return s->bind();
}

int RShaderManager::flush()
{
    std::map <SHADER_TYPE, RShader *>::iterator it;
    DEBUG_NOTIFICATION("Deleting all cached shaders from shaderCache\n");
    for (it=shaderCache.begin(); it != shaderCache.end(); ++it)
    {
        delete (*it).second;
    }
    shaderCache.clear();
    return 0;
}

int RShaderManager::setTransformation(Matrix4X4 *m)
{
    std::map <SHADER_TYPE, RShader *>::iterator it;
    DEBUG_NOTIFICATION("Setting transformation uniform for currently active shader\n");
    for (it=shaderCache.begin(); it != shaderCache.end(); ++it)
    {
        if (currentProgramObject == (*it).second->getProgramObject())
        {
            (*it).second->setTransformation(m);
            return 0;
        }
    }
    return -1;
}

int RShaderManager::setTextureUnit(GLuint texunit)
{
    std::map <SHADER_TYPE, RShader *>::iterator it;
    DEBUG_NOTIFICATION("Trying to set active texture unit uniform\n");
    for (it=shaderCache.begin(); it != shaderCache.end(); ++it)
    {
        if (currentProgramObject ==(*it).second->getProgramObject())
        {
            (*it).second->setTextureUnit(texunit);
            return 0;
        }
    }
    return -1;
}

int RShaderManager::setAmbientColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a)
{
    std::map <SHADER_TYPE, RShader *>::iterator it;
    DEBUG_NOTIFICATION("Trying to set ambient color uniform\n");
    for (it=shaderCache.begin(); it != shaderCache.end(); ++it)
    {
        if (currentProgramObject ==(*it).second->getProgramObject())
        {
            (*it).second->setAmbientColor(r, g, b, a);
            return 0;
        }
    }
    return -1;
}

int RShaderManager::setDiffuseColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a)
{
    std::map <SHADER_TYPE, RShader *>::iterator it;
    DEBUG_NOTIFICATION("Trying to set diffuse color uniform\n");
    for (it=shaderCache.begin(); it != shaderCache.end(); ++it)
    {
        if (currentProgramObject ==(*it).second->getProgramObject())
        {
            (*it).second->setDiffuseColor(r, g, b, a);
            return 0;
        }
    }
    return -1;
}

int RShaderManager::setSpecularColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a)
{
    std::map <SHADER_TYPE, RShader *>::iterator it;
    DEBUG_NOTIFICATION("Trying to set specular color uniform\n");
    for (it=shaderCache.begin(); it != shaderCache.end(); ++it)
    {
        if (currentProgramObject ==(*it).second->getProgramObject())
        {
            (*it).second->setSpecularColor(r, g, b, a);
            return 0;
        }
    }
    return -1;
}

int RShaderManager::setEmissiveColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a)
{
    std::map <SHADER_TYPE, RShader *>::iterator it;
    DEBUG_NOTIFICATION("Trying to set emissive color uniform\n");
    for (it=shaderCache.begin(); it != shaderCache.end(); ++it)
    {
        if (currentProgramObject ==(*it).second->getProgramObject())
        {
            (*it).second->setEmissiveColor(r, g, b, a);
            return 0;
        }
    }
    return -1;
}

int RShaderManager::setSpecularPower(GLfloat power)
{
    std::map <SHADER_TYPE, RShader *>::iterator it;
    DEBUG_NOTIFICATION("Trying to set specular power uniform\n");
    for (it=shaderCache.begin(); it != shaderCache.end(); ++it)
    {
        if (currentProgramObject ==(*it).second->getProgramObject())
        {
            (*it).second->setSpecularPower(power);
            return 0;
        }
    }
    return -1;
}

