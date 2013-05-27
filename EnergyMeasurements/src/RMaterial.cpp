
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "RMaterial.h"
#include "GLWrapper.h"
#include "DebugLog.h"

#include "RShaderManager.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>

RMaterial::RMaterial(int id) :
    amb_r(0.0f),
    amb_g(0.0f),
    amb_b(0.0f),
    spec_r(0.0f),
    spec_g(0.0f),
    spec_b(0.0f),
    diff_r(1.0f),
    diff_g(1.0f),
    diff_b(1.0f),
    emis_r(0.0f),
    emis_g(0.0f),
    emis_b(0.0f),
    specularPower(32.0f),
    alphaRejection(0.0f),
    n_textures(0)
{
    ID = id;

    // Make sure all params are reset
    memset(name, 0, sizeof(name));
    memset(textures, 0, sizeof(textures));
    for (unsigned int i=0; i<4; i++)
    {
        tex_addressingmode[i] = MAT_AMODE_CLAMP;
        tex_filter[i] = MAT_FILTER_NONE;
    }
    alphaRejectionMode = MAT_AREJ_ALWAYSPASS;
}

RMaterial::~RMaterial()
{
    destroy();
}

int RMaterial::destroy(void)
{
    return 0;
}

/******************************************************************************
 * Resouce API
 */

int RMaterial::bind(void)
{
    unsigned int i;
    Resource *t;
    GLuint t_units[] = { GL_TEXTURE0, GL_TEXTURE1, GL_TEXTURE2, GL_TEXTURE3 };

    // First we bind the correct shader program to match the current material patamaters:
    DEBUG_NOTIFICATION("Selecting shader program\n");
    switch(n_textures)
    {
    default:
        DEBUG_INFO("%d textures defined for this material. No shader available\n", n_textures);
        return -1;
    case 0:
        DEBUG_NOTIFICATION("Submesh has 0 textures, using ambient only shader\n", SHADER_SIMPLE);
        RShaderManager::Instance()->bind(SHADER_SIMPLE);
        break;
    case 1:
        DEBUG_NOTIFICATION("Submesh has 1 texture, using predefined shader %d\n", SHADER_TEXTUREMAP_1CHANNEL);
        RShaderManager::Instance()->bind(SHADER_TEXTUREMAP_1CHANNEL);
        break;
    case 2:
        DEBUG_NOTIFICATION("Submesh has 2 textures, using predefined shader %d\n", SHADER_TEXTUREMAP_2CHANNEL_LIGHTMAP);
        RShaderManager::Instance()->bind(SHADER_TEXTUREMAP_2CHANNEL_LIGHTMAP);
        break;
    case 3:
        DEBUG_NOTIFICATION("Submesh has 3 textures, using predefined shader %d\n", SHADER_TEXTUREMAP_3CHANNEL);
        RShaderManager::Instance()->bind(SHADER_TEXTUREMAP_3CHANNEL);
        break;
    case 4:
        DEBUG_NOTIFICATION("Submesh has 2 textures, using predefined shader %d\n", SHADER_TEXTUREMAP_4CHANNEL);
        RShaderManager::Instance()->bind(SHADER_TEXTUREMAP_4CHANNEL);
        break;
    }

    // Then we bind required uniforms for the shader
    DEBUG_NOTIFICATION("Binding uniforms for the shader\n");
    RShaderManager::Instance()->setAmbientColor(amb_r, amb_g, amb_b, 1.0f);
    RShaderManager::Instance()->setDiffuseColor(diff_r, diff_g, diff_b, 1.0f);
    RShaderManager::Instance()->setSpecularColor(spec_r, spec_g, spec_b, 1.0f);
    RShaderManager::Instance()->setEmissiveColor(emis_r, emis_g, emis_b, 1.0f);
    RShaderManager::Instance()->setSpecularPower(specularPower);

    // Finally we bind the textures for the shader.
    DEBUG_NOTIFICATION("Binding textures for the shader\n");
    for (i=0; i<n_textures; i++)
    {
        t = ResourceManager::Instance()->getResourceByName(textures[i]);
        DEBUG_NOTIFICATION("Received texture resource pointer %p\n", t);
        if (t == NULL) continue;
        GLWrapper::Instance()->GLACTIVETEXTURE(t_units[i]);
        t->bind();
        RShaderManager::Instance()->setTextureUnit(t_units[i]);

        // Material specific texture params:
        switch(tex_filter[i])
        {
        case MAT_FILTER_NONE:
            DEBUG_NOTIFICATION("Applying none texture filtering\n");
            GLWrapper::Instance()->GLTEXPARAMETERI(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            GLWrapper::Instance()->GLTEXPARAMETERI(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            break;
        case MAT_FILTER_BILINEAR:
            DEBUG_NOTIFICATION("Applying bi-linear texture filtering\n");
            GLWrapper::Instance()->GLTEXPARAMETERI(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
            GLWrapper::Instance()->GLTEXPARAMETERI(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            break;
        default:
        case MAT_FILTER_ANISOTROPIC: // Anisotropic filtering defaults to tri-linear for now
        case MAT_FILTER_TRILINEAR:
            DEBUG_NOTIFICATION("Applying tri-linear texture filtering\n");
            GLWrapper::Instance()->GLTEXPARAMETERI(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            GLWrapper::Instance()->GLTEXPARAMETERI(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            break;
        }

        switch(tex_addressingmode[i])
        {
        case MAT_AMODE_WRAP:
            DEBUG_NOTIFICATION("Applying wrap texture addressing mode\n");
            GLWrapper::Instance()->GLTEXPARAMETERI(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            GLWrapper::Instance()->GLTEXPARAMETERI(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            break;
        case MAT_AMODE_MIRROR:
            DEBUG_NOTIFICATION("Applying mirrored texture addressing mode\n");
            GLWrapper::Instance()->GLTEXPARAMETERI(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
            GLWrapper::Instance()->GLTEXPARAMETERI(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
            break;
        default:
        case MAT_AMODE_BORDER: // Border addressing mode defaults to CLAMP for now
        case MAT_AMODE_CLAMP:
            DEBUG_NOTIFICATION("Applying clamp texture addressing mode\n");
            GLWrapper::Instance()->GLTEXPARAMETERI(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            GLWrapper::Instance()->GLTEXPARAMETERI(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            break;
        }
    }
    return 0;
}

int RMaterial::render(float deltatime)
{
    return 0;
}

/******************************************************************************
 * Public RMaterial methods
 */

int RMaterial::setTextureName(int order, const char *tex)
{
    if (order < 0 || order >= 4)
    {
        DEBUG_WARNING("Trying to set texture to material index %d, which is out of scale 0->3\n", order);
        return -1;
    }
    DEBUG_NOTIFICATION("Adding texture ref %s to material texture channel %d (stringsize=%d bytes)\n", tex, order, sizeof(textures[order]));
    memset(&textures[order][0], 0, sizeof(textures[order]));
    memcpy(&textures[order][0], tex, strlen(tex));
    n_textures = countTextureChannels();
    return 0;
}

int RMaterial::setTextureAddressingmode(int order, MAT_AMODE mode)
{
    if (order < 0 || order >= 4)
    {
        DEBUG_WARNING("Trying to set texture addressing mode to out-of-range texture (%d)\n", order);
        return -1;
    }
    switch(mode)
    {
    case MAT_AMODE_WRAP:
    case MAT_AMODE_CLAMP:
    case MAT_AMODE_MIRROR:
    case MAT_AMODE_BORDER:
        DEBUG_NOTIFICATION("Setting material addressing mode for texture %d, mode %d\n", order, mode);
        tex_addressingmode[order] = mode;
        break;
    default:
        DEBUG_WARNING("Unrecognized texture addressing param (%d)\n", mode);
        return -1;
    }
    return 0;
}

int RMaterial::setTextureFilter(int order, MAT_FILTER filter)
{
    if (order < 0 || order >= 4)
    {
        DEBUG_WARNING("Trying to set texture filter to out-of-range texture (%d)\n", order);
        return -1;
    }
    switch(filter)
    {
    case MAT_FILTER_NONE:
    case MAT_FILTER_BILINEAR:
    case MAT_FILTER_TRILINEAR:
    case MAT_FILTER_ANISOTROPIC:
        DEBUG_NOTIFICATION("Setting material filtering for texture %d, mode %d\n", order, filter);
        tex_filter[order] = filter;
        break;
    default:
        DEBUG_WARNING("Unrecognized texture filtering mode (%d)\n", filter);
        return -1;
    }
    return 0;
}

int RMaterial::setAlphaRejection(MAT_AREJ mode, float limit)
{
    switch(mode)
    {
    case MAT_AREJ_ALWAYSPASS:
    case MAT_AREJ_ALWAYSREJECT:
    case MAT_AREJ_LESS:
    case MAT_AREJ_LESSEQUAL:
    case MAT_AREJ_EQUAL:
    case MAT_AREJ_NOTEQUAL:
    case MAT_AREJ_GREATEREQUAL:
    case MAT_AREJ_GREATER:
        DEBUG_NOTIFICATION("Setting material alpha rejection, mode %d, limit %f\n", mode, limit);
        alphaRejectionMode = mode;
        break;
    default:
        DEBUG_WARNING("Unrecognized alpha rejection mode (%d)\n", mode);
        return -1;
    }
    return 0;
}

/******************************************************************************
 * Private methods
 */

int RMaterial::countTextureChannels(void)
{
    int count = 0;
    if (strlen(&textures[0][0]) > 0) count++;
    if (strlen(&textures[1][0]) > 0) count++;
    if (strlen(&textures[2][0]) > 0) count++;
    if (strlen(&textures[3][0]) > 0) count++;
    DEBUG_INFO("Currently %d active texture channels in the material\n", count);
    return count;
}
