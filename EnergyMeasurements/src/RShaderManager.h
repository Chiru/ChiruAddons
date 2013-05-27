
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef RShaderManager_H
#define RShaderManager_H

#include "Neocortex_GLHeaders.h"

#include "GLMath.h"

#include <map>

class RShader;

typedef enum {
    SHADER_NONE = 0,
    SHADER_SIMPLE,
    SHADER_TEXTUREMAP_1CHANNEL,
    SHADER_TEXTUREMAP_2CHANNEL_LIGHTMAP,
    SHADER_TEXTUREMAP_3CHANNEL,
    SHADER_TEXTUREMAP_4CHANNEL
} SHADER_TYPE;

typedef struct _shaderlookuptable {
    SHADER_TYPE type;
    const char * vertexshader;
    const char * fragmentshader;
} SHADERLOOKUPTABLE;


/// Class definition
class RShaderManager
{
public:
    static RShaderManager * Instance();
    ~RShaderManager();

    /// Activate a specific shader
    int bind(SHADER_TYPE type);

    /// Flush all shaders from memory
    int flush();

    /// Set world transformation uniform
    int setTransformation(Matrix4X4 *m);

    /// Set active textureunit uniform sampler
    int setTextureUnit(GLuint texunit);

    /// Set color uniforms
    int setAmbientColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a);
    int setDiffuseColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a);
    int setSpecularColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a);
    int setEmissiveColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a);

    /// Special parameters
    int setSpecularPower(GLfloat power);

protected:

private:
    // RShadermanager singleton
    RShaderManager() { currentProgramObject = 0; }        // Private constructor
    RShaderManager(RShaderManager const &) {}
    static RShaderManager *p_Instance;                    // Single instance placeholder

    // Shader cache
    std::map <SHADER_TYPE, RShader *> shaderCache;

    // Currently bound shader program object
    GLuint currentProgramObject;
};

#endif // RShaderManager_H
