
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef RShader_H
#define RShader_H

#include "Neocortex_GLHeaders.h"


/// Class definition
class RShader
{
public:
    RShader();
    ~RShader();

    int bind(void);
    int destroy(void);

    int fromVectors(char *v_src, char *f_src);

    GLuint getProgramObject(void) const { return programObject; }

    int setTransformation(Matrix4X4 *m);
    int setTextureUnit(GLint texunit);

    int setAmbientColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a);
    int setDiffuseColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a);
    int setSpecularColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a);
    int setEmissiveColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a);

    int setSpecularPower(GLfloat power);

protected:

private:
    // Helper methods for shader handling
    GLuint createShaderProgram(const char *v_src, const char *f_src);
    GLuint loadShaderProgram(const char *shader_source, GLenum type);
    void linkShaderProgram(GLuint shaderProgram);
    bool printShaderInfo(GLuint shader);

    GLuint vertexShader;
    GLuint fragmentShader;
    GLuint programObject;

    GLint u_WorldTransform;
    GLint u_Texture0;
    GLint u_Texture1;
    GLint u_Texture2;
    GLint u_Texture3;
    GLint u_Ambient;
    GLint u_Diffuse;
    GLint u_Specular;
    GLint u_Emissive;
    GLint u_SpecularPower;

    GLchar shaderInfo[512];
};

#endif // RShader_H
