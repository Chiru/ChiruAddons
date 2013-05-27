
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "GLMath.h"
#include "RShaderManager.h"
#include "RShader.h"
#include "GLWrapper.h"
#include "DebugLog.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>


RShader::RShader() :
    vertexShader(0),
    fragmentShader(0),
    programObject(0)
{
}

RShader::~RShader()
{
    destroy();
}

/******************************************************************************
 * I/O
 */

int RShader::fromVectors(char *v_src, char *f_src)
{
    programObject = createShaderProgram(v_src, f_src);
    if (programObject == 0)
    {
        DebugLog::Instance()->MESSAGE(1, "Shader program object creation failed\n");
        return -1;
    }
    GLWrapper::Instance()->GLLINKPROGRAM(programObject);

    // After linking, read back uniform variable locations
    u_WorldTransform = GLWrapper::Instance()->GLGETUNIFORMLOCATION(getProgramObject(), "u_WorldTransform");
    u_Texture0       = GLWrapper::Instance()->GLGETUNIFORMLOCATION(getProgramObject(), "s_Texture0");
    u_Texture1       = GLWrapper::Instance()->GLGETUNIFORMLOCATION(getProgramObject(), "s_Texture1");
    u_Texture2       = GLWrapper::Instance()->GLGETUNIFORMLOCATION(getProgramObject(), "s_Texture2");
    u_Texture3       = GLWrapper::Instance()->GLGETUNIFORMLOCATION(getProgramObject(), "s_Texture3");
    u_Ambient        = GLWrapper::Instance()->GLGETUNIFORMLOCATION(getProgramObject(), "u_Ambient");
    u_Diffuse        = GLWrapper::Instance()->GLGETUNIFORMLOCATION(getProgramObject(), "u_Diffuse");
    u_Specular       = GLWrapper::Instance()->GLGETUNIFORMLOCATION(getProgramObject(), "u_Specular");
    u_Emissive       = GLWrapper::Instance()->GLGETUNIFORMLOCATION(getProgramObject(), "u_Emissive");
    u_SpecularPower  = GLWrapper::Instance()->GLGETUNIFORMLOCATION(getProgramObject(), "u_SpecularPower");

    return bind();
}

int RShader::bind(void)
{
    if (programObject != 0)
    {
        DEBUG_NOTIFICATION("Binding shader program %d\n", programObject);
        GLWrapper::Instance()->GLUSEPROGRAM(programObject);
        return 0;
    }
    return -1;
}

int RShader::destroy(void)
{
    if (programObject == 0)
    {
        return -1;
    }
    GLWrapper::Instance()->GLDELETEPROGRAM(programObject);
    GLWrapper::Instance()->GLDELETESHADER(vertexShader);
    GLWrapper::Instance()->GLDELETESHADER(fragmentShader);
    programObject = 0;
    return 0;
}

/******************************************************************************
 * Private methods
 */

/*
 * Shader program helpers
 * ----------------------
 */

bool RShader::printShaderInfo ( GLuint shader )
{
   GLint length;

   GLWrapper::Instance()->GLGETSHADERIV( shader , GL_INFO_LOG_LENGTH , &length );

   if ( length )
   {
      GLint success;

      GLWrapper::Instance()->GLGETSHADERINFOLOG( shader , length , NULL , shaderInfo );
      DEBUG_NOTIFICATION("Shader info: '%s'\n", shaderInfo);

      GLWrapper::Instance()->GLGETSHADERIV( shader, GL_COMPILE_STATUS, &success );
      if ( success != GL_TRUE )
      {
          DEBUG_NOTIFICATION("Error: Shader compilation failed\n");
          return false;
      }
   }
   return true;
}

GLuint RShader::loadShaderProgram ( const char *shader_source, GLenum type)
{
   GLuint shader;

   shader = GLWrapper::Instance()->GLCREATESHADER( type );
   if (shader == 0)
   {
       DebugLog::Instance()->MESSAGE(2, "loadShader: shader creation failed.\n");
       return 0;
   }

   GLWrapper::Instance()->GLSHADERSOURCE( shader , 1 , &shader_source , NULL );
   GLWrapper::Instance()->GLCOMPILESHADER( shader );

   if (false == printShaderInfo ( shader ))
   {
       return 0;
   }

   return shader;
}

GLuint RShader::createShaderProgram(const char *v_src, const char *f_src)
{
    GLuint shaderProgram;

    shaderProgram = GLWrapper::Instance()->GLCREATEPROGRAM();
    if (shaderProgram == 0)
    {
        DebugLog::Instance()->MESSAGE(1, "Error: Shader program creation failed\n");
        return 0;
    }

    DEBUG_NOTIFICATION("Initializing shaders...\n");
    if (v_src == NULL || f_src == NULL)
    {
        DEBUG_NOTIFICATION("Vertex nor fragment shader source must not be NULL.\n");
        return 0;
    }

    vertexShader   = loadShaderProgram ( v_src , GL_VERTEX_SHADER  );
    fragmentShader = loadShaderProgram ( f_src , GL_FRAGMENT_SHADER );
    if (vertexShader == 0 || fragmentShader == 0)
    {
        DebugLog::Instance()->MESSAGE(1, "Error: Shader program loading failed\n");
        return 0;
    }

    GLWrapper::Instance()->GLATTACHSHADER(shaderProgram, vertexShader);
    GLWrapper::Instance()->GLATTACHSHADER(shaderProgram, fragmentShader);

    /*
     * Now the program has both vertex and fragment programs attached, but it is not
     * linked yet. Based on shader API, attributes are bound at this stage, before linking.
     * Uniforms IDs are then queried after the link process
     */
    GLWrapper::Instance()->GLBINDATTRIBLOCATION(shaderProgram, 0, "a_Position");
    GLWrapper::Instance()->GLBINDATTRIBLOCATION(shaderProgram, 1, "a_Texcoord");
    GLWrapper::Instance()->GLBINDATTRIBLOCATION(shaderProgram, 2, "a_Normal");
    GLWrapper::Instance()->GLBINDATTRIBLOCATION(shaderProgram, 3, "a_Color");

    return shaderProgram;
}

int RShader::setTransformation(Matrix4X4 *m)
{
    DEBUG_NOTIFICATION("Setting transformation matrix for shader\n");
    GLWrapper::Instance()->GLUNIFORMMATRIX4FV(u_WorldTransform, 1, GL_FALSE, (GLfloat *)m);
    return 0;
}

int RShader::setTextureUnit(GLint texunit)
{
    switch(texunit)
    {
    case GL_TEXTURE0:
        DEBUG_NOTIFICATION("Setting activation uniform for texture unit 0, s_Texture0, uniformlocation %d\n", u_Texture0);
        if (u_Texture0 != -1)
            GLWrapper::Instance()->GLUNIFORM1I(u_Texture0, 0);
        break;
    case GL_TEXTURE1:
        DEBUG_NOTIFICATION("Setting activation uniform for texture unit 1, s_Texture1, uniformlocation %d\n", u_Texture1);
        if (u_Texture1 != -1)
            GLWrapper::Instance()->GLUNIFORM1I(u_Texture1, 1);
        break;
    case GL_TEXTURE2:
        DEBUG_NOTIFICATION("Setting activation uniform for texture unit 2, s_Texture2, uniformlocation %d\n", u_Texture2);
        if (u_Texture2 != -1)
            GLWrapper::Instance()->GLUNIFORM1I(u_Texture2, 2);
        break;
    case GL_TEXTURE3:
        DEBUG_NOTIFICATION("Setting activation uniform for texture unit 3, s_Texture3, uniformlocation %d\n", u_Texture3);
        if (u_Texture3 != -1)
            GLWrapper::Instance()->GLUNIFORM1I(u_Texture3, 3);
        break;
    default:
        DEBUG_FATAL("Trying to set illegal texture activation uniform %d\n", texunit);
        return -1;
    }
    return 0;
}

int RShader::setAmbientColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a)
{
    if (u_Ambient != -1)
    {
        DEBUG_NOTIFICATION("Setting Ambient color to %f %f %f %f\n", r, g, b, a);
        GLWrapper::Instance()->GLUNIFORM4F(u_Ambient, r, g, b, a);
        return 0;
    }
    return -1;
}

int RShader::setDiffuseColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a)
{
    if (u_Diffuse != -1)
    {
        DEBUG_NOTIFICATION("Setting Diffuse color to %f %f %f %f\n", r, g, b, a);
        GLWrapper::Instance()->GLUNIFORM4F(u_Diffuse, r, g, b, a);
        return 0;
    }
    return -1;
}

int RShader::setSpecularColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a)
{
    if (u_Ambient != -1)
    {
        DEBUG_NOTIFICATION("Setting Specular color to %f %f %f %f\n", r, g, b, a);
        GLWrapper::Instance()->GLUNIFORM4F(u_Specular, r, g, b, a);
        return 0;
    }
    return -1;
}

int RShader::setEmissiveColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a)
{
    if (u_Ambient != -1)
    {
        DEBUG_NOTIFICATION("Setting Emissive color to %f %f %f %f\n", r, g, b, a);
        GLWrapper::Instance()->GLUNIFORM4F(u_Emissive, r, g, b, a);
        return 0;
    }
    return -1;
}

int RShader::setSpecularPower(GLfloat power)
{
    if (u_Ambient != -1)
    {
        DEBUG_NOTIFICATION("Setting Specular power to %f\n", power);
        GLWrapper::Instance()->GLUNIFORM1F(u_SpecularPower, power);
        return 0;
    }
    return -1;
}
