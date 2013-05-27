
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */


#include "Neocortex_GLHeaders.h"
#include "GLWrapper.h"
#include "DebugLog.h"

#include <iostream>


#if defined(_DEBUG)
#define FLUSH_GL_ERRORS() flushGLErrors();
#define FLUSH_EGL_ERRORS() flushEGLErrors();
#else
#define FLUSH_GL_ERRORS()
#define FLUSH_EGL_ERRORS()
#endif

#ifndef GL_STACK_OVERFLOW
#define GL_STACK_OVERFLOW 0x0503
#endif
#ifndef GL_STACK_UNDERFLOW
#define GL_STACK_UNDERFLOW 0x0504
#endif

/******************************************************************************
 * Global static instance of the class
 */

GLWrapper * GLWrapper::p_Instance = NULL;

GLWrapper * GLWrapper::Instance()
{
    if (p_Instance == NULL)
    {
        p_Instance = new GLWrapper();
    }
    return p_Instance;
}

/*
 * GL Wrappers:
 * ------------
 * The intention of these methods are to easily wrap GL calls and bundle them with something
 * useful, like debug information and error grabbing. This way seems most suitable for the
 * purpose.
 */

GLWrapper::GLWrapper() :
    GLerrors(0),
    EGLerrors(0)
{
}

GLWrapper::~GLWrapper()
{
}

/*
 *
 */

#if defined(_DEBUG)

void GLWrapper::flushGLErrors(void)
{
    GLuint error;
    error = GLWrapper::Instance()->GLGETERROR();
    if (error != GL_NO_ERROR)
    {
        GLerrors++;
        DEBUG_FATAL("GL ERROR: glGetError() returned %d (0x%04x)\n", error, error);
        switch(error)
        {
        case GL_INVALID_ENUM:       // 0x0500
            DEBUG_FATAL(" (GL_INVALID_ENUM)\n");
            break;
        case GL_INVALID_VALUE:      // 0x0501
            DEBUG_FATAL(" (GL_INVALID_VALUE)\n");
            break;
        case GL_INVALID_OPERATION:  // 0x0502
            DEBUG_FATAL(" (GL_INVALID_OPERATION)\n");
            break;
        case GL_STACK_OVERFLOW:     // 0x0503
            DEBUG_FATAL(" (GL_STACK_OVERFLOW)\n");
            break;
        case GL_STACK_UNDERFLOW:    // 0x0504
            DEBUG_FATAL(" (GL_STACK_UNDERFLOW)\n");
            break;
        case GL_OUT_OF_MEMORY:      // 0x0505
            DEBUG_FATAL(" (GL_OUT_OF_MEMORY)\n");
            break;
        default:
            DEBUG_FATAL(" (UNKNOWN GL ERROR)\n");
            break;
#if !defined(NC_WIN32) // glCheckFramebufferstatus is not defined for Win32
        case GL_INVALID_FRAMEBUFFER_OPERATION: // 0x0506
            DEBUG_FATAL(" (GL_INVALID_FRAMEBUFFER_OPERATION) \n");
            DEBUG_FATAL(" glCheckFramebufferStatus(GL_FRAMEBUFFER) = %d\n", glCheckFramebufferStatus(GL_FRAMEBUFFER));
            break;
#endif
        }
    }
}

void GLWrapper::flushEGLErrors(void)
{
#if defined(NC_EGL)
    GLuint error;
    error = eglGetError();
    if (error != EGL_SUCCESS)
    {
        EGLerrors++;
        std::cout << "EGL ERROR: eglGetError() returned " << error;
        switch(error)
        {
        case EGL_NOT_INITIALIZED:
            std::cout << " (EGL_NOT_INITIALIZED)\n";
            break;
        case EGL_BAD_ACCESS:
            std::cout << " (EGL_BAD_ACCESS)\n";
            break;
        case EGL_BAD_ALLOC:
            std::cout << " (EGL_BAD_ALLOC)\n";
            break;
        case EGL_BAD_ATTRIBUTE:
            std::cout << " (EGL_BAD_ATTRIBUTE) ";
            break;
        case EGL_BAD_CONTEXT:
            std::cout << " (EGL_BAD_CONTEXT) ";
            break;
        case EGL_BAD_CONFIG:
            std::cout << " (EGL_BAD_CONFIG) ";
            break;
        case EGL_BAD_CURRENT_SURFACE:
            std::cout << " (EGL_BAD_CURRENT_SURFACE) ";
            break;
        case EGL_BAD_DISPLAY:
            std::cout << " (EGL_BAD_DISPLAY) ";
            break;
        case EGL_BAD_SURFACE:
            std::cout << " (EGL_BAD_SURFACE) ";
            break;
        case EGL_BAD_MATCH:
            std::cout << " (EGL_BAD_MATCH) ";
            break;
        case EGL_BAD_PARAMETER:
            std::cout << " (EGL_BAD_PARAMETER) ";
            break;
        case EGL_BAD_NATIVE_PIXMAP:
            std::cout << " (EGL_BAD_NATIVE_PIXMAP) ";
            break;
        case EGL_BAD_NATIVE_WINDOW:
            std::cout << " (EGL_BAD_NATIVE_WINDOW) ";
            break;
        default:
            std::cout << " (UNKNOWN EGL ERROR)\n";
            break;
        }
    }
#endif
}

#endif

/*
 *
 */

void GLWrapper::GLATTACHSHADER(GLuint shaderProgram, GLuint shader)
{
    DEBUG_NOTIFICATION("GL call: glAttachShader(%d, %d)\n", shaderProgram, shader);
    glAttachShader(shaderProgram, shader);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLBINDATTRIBLOCATION(GLuint shaderProgram, GLuint index, const GLchar *name)
{
    DEBUG_NOTIFICATION("GL call: glBindAttribLocation(%d, %d, '%s')\n", shaderProgram, index, name);
    glBindAttribLocation(shaderProgram, index, name);
    FLUSH_GL_ERRORS();
}
void GLWrapper::GLCLEARCOLOR(GLclampf r, GLclampf g, GLclampf b, GLclampf a)
{
    DEBUG_NOTIFICATION("GL call: glClearColor(%f, %f, %f, %f)\n", r, g, b, a);
    glClearColor(r, g, b, a);
    FLUSH_GL_ERRORS();
}

GLuint GLWrapper::GLCREATEPROGRAM(void)
{
    GLuint program;
    program = glCreateProgram();
    DEBUG_NOTIFICATION("GL call: glCreateProgram() = %d\n", program);
    FLUSH_GL_ERRORS();
    return program;
}

void GLWrapper::GLLINKPROGRAM(GLuint program)
{
    DEBUG_NOTIFICATION("GL call: glLinkProgram(%d)\n", program);
    glLinkProgram(program);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLUSEPROGRAM(GLuint program)
{
    DEBUG_INFO("GL call: glUseProgram(%d)\n", program);
    GFXDEBUG_SHADERBINDING();
    glUseProgram(program);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLVIEWPORT(GLint x, GLint y, GLsizei width, GLsizei height)
{
    DEBUG_INFO("GL call: glViewPort(%d, %d, %d, %d)\n", x, y, width, height);
    glViewport(x, y, width, height);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLCLEAR(GLbitfield mask)
{
    DEBUG_INFO("GL call: glClear(0x%x)\n", mask);
    glClear(mask);
    FLUSH_GL_ERRORS();
}

#if 0
struct _vap {
    GLint size;
    GLenum type;
    GLboolean normalized;
    GLsizei stride;
    const GLvoid *data;
};
static _vap __cached_vertexattribpointers[8] = {0,};
void GLWrapper::GLVERTEXATTRIBPOINTER(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const GLvoid *data)
{
    if (__cached_vertexattribpointers[index].size != size)
        goto setvertexattribpointer;
    if (__cached_vertexattribpointers[index].type != type)
        goto setvertexattribpointer;
    if (__cached_vertexattribpointers[index].normalized != normalized)
        goto setvertexattribpointer;
    if (__cached_vertexattribpointers[index].stride != stride)
        goto setvertexattribpointer;
    if (__cached_vertexattribpointers[index].data != data)
        goto setvertexattribpointer;

#ifdef _DEBUG
    DEBUG_INFO("GL call: Used cached values %d %d %d %d %p for glVertexAttribArray(), index %d\n", size, type, normalized, stride, data, index);
#endif
    return;

setvertexattribpointer:
    DEBUG_INFO("GL call: glVertexAttribPointer(%d, %d, %d, %d, %d, %p)\n",
               index, size, type, normalized, stride, data);
    glVertexAttribPointer(index, size, type, normalized, stride, data);
    FLUSH_GL_ERRORS();
    __cached_vertexattribpointers[index].size = size;
    __cached_vertexattribpointers[index].type = type;
    __cached_vertexattribpointers[index].normalized = normalized;
    __cached_vertexattribpointers[index].stride = stride;
    __cached_vertexattribpointers[index].data = data;
}
#else
void GLWrapper::GLVERTEXATTRIBPOINTER(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const GLvoid *data)
{
    DEBUG_INFO("GL call: glVertexAttribPointer(%d, %d, %d, %d, %d, %p)\n",
               index, size, type, normalized, stride, data);
    glVertexAttribPointer(index, size, type, normalized, stride, data);
    FLUSH_GL_ERRORS();
}
#endif

static GLuint __cached_vertexattribarray[8] = {0, 0, 0, 0, 0, 0, 0, 0};
void GLWrapper::GLENABLEVERTEXATTRIBARRAY(GLuint index)
{
    if (__cached_vertexattribarray[index] == 0)
    {
        DEBUG_INFO("GL call: glEnableVertexAttribArray(%d)\n", index);
        glEnableVertexAttribArray(index);
        FLUSH_GL_ERRORS();
        __cached_vertexattribarray[index] = 1;
    }
#ifdef _DEBUG
    else
    {
        DEBUG_INFO("Cached GL call: glEnableVertexAttribArray(%d)\n", index);
    }
#endif
}

void GLWrapper::GLDISABLEVERTEXATTRIBARRAY(GLuint index)
{
    if (__cached_vertexattribarray[index] == 1)
    {
        DEBUG_INFO("GL call: glDisableVertexAttribArray(%d)\n", index);
        glDisableVertexAttribArray(index);
        FLUSH_GL_ERRORS();
        __cached_vertexattribarray[index] = 0;
    }
#ifdef _DEBUG
    else
    {
        DEBUG_INFO("Cached GL call: glDisableVertexAttribArray(%d)\n", index);
    }
#endif
}

void GLWrapper::GLDRAWARRAYS(GLenum mode, GLint first, GLsizei size)
{
    DEBUG_INFO("GL call: glDrawArrays(%d, %d, %d)\n", mode, first, size);
    glDrawArrays(mode, first, size);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLGENTEXTURES(GLsizei size, GLuint *ptr)
{
    DEBUG_NOTIFICATION("GL call: glGenTextures(%d, %p)\n", size, ptr);
    glGenTextures(size, ptr);
    FLUSH_GL_ERRORS();
}

static GLenum __cached_bindtexture_target = -1;
static GLuint __cached_bindtexture_id = -1;
void GLWrapper::GLBINDTEXTURE(GLenum target, GLuint id)
{
    if (__cached_bindtexture_target != target || __cached_bindtexture_id != id)
    {
        DEBUG_INFO("GL call: glBindTexture(%d, %d)\n", target, id);
        glBindTexture(target, id);
        FLUSH_GL_ERRORS();
        __cached_bindtexture_target = target;
        __cached_bindtexture_id = id;
    }
#ifdef _DEBUG
    else
    {
        DEBUG_INFO("Cached GL call: glBindTexture(%d, %d)\n", target, id);
    }
#endif
    GFXDEBUG_TEXTUREBINDING(id);
}

void GLWrapper::GLPIXELSTOREI(GLenum type, GLint align)
{
    DEBUG_NOTIFICATION("GL call: glPixelStorei(%d, %d)\n", type, align);
    glPixelStorei(type, align);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLTEXIMAGE2D(GLenum target, GLint level, GLint internalformat, GLsizei width,
                                   GLsizei height, GLint border, GLenum format, GLenum type, const void *pixels)
{
    DEBUG_NOTIFICATION("GL call: glTexImage2D(%d, %d, %d, %d, %d, %d, %d, %d, %p)\n",
               target, level, internalformat, width, height, border, format, type, pixels);
    glTexImage2D(target, level, internalformat, width, height, border, format, type, pixels);
    FLUSH_GL_ERRORS();
#if defined(_NDEBUG)
    int size;
    switch(format)
    {
        case GL_RGB: size = 3; break;
        case GL_RGBA: size = 4; break;
        default:
        case GL_UNSIGNED_BYTE: size = 1; break;
    }
    size *= (width*height);
#endif
    GFXDEBUG_ADDRGBTEXTUREDATA(size);
}

void GLWrapper::GLTEXPARAMETERI(GLenum target, GLenum pname, GLint param)
{
    DEBUG_NOTIFICATION("GL call: glTexParameteri(%d, %d, %d)\n", target, pname, param);
    glTexParameteri(target, pname, param);
    FLUSH_GL_ERRORS();
}

GLint GLWrapper::GLGETATTRIBLOCATION(GLuint program, const GLchar *name)
{
    GLint rc;
    rc = glGetAttribLocation(program, name);
    DEBUG_NOTIFICATION("GL call: glGetAttribLocation(%d, '%s') = %d\n", program, name, rc);
    FLUSH_GL_ERRORS();
    return rc;
}

GLint GLWrapper::GLGETUNIFORMLOCATION(GLuint program, const GLchar *name)
{
    GLint rc;
    rc = glGetUniformLocation(program, name);
    DEBUG_NOTIFICATION("GL call: glGetUniformLocation(%d, '%s') = %d\n", program, name, rc);
    FLUSH_GL_ERRORS();
    return rc;
}

static GLenum __cached_activetexture = -1;
void GLWrapper::GLACTIVETEXTURE(GLenum texture)
{
    if (texture != __cached_activetexture)
    {
        DEBUG_INFO("GL call: glActiveTexture(%d)\n", texture);
        glActiveTexture(texture);
        FLUSH_GL_ERRORS();
        __cached_activetexture = texture;
    }
#ifdef _DEBUG
    else
    {
        DEBUG_INFO("Cached GL call: glActiveTexture(%d)\n", texture);
    }
#endif
}

void GLWrapper::GLUNIFORM1I(GLint location, GLint x)
{
    DEBUG_INFO("GL call: glUniform1i(%d, %d)\n", location, x);
    glUniform1i(location, x);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLUNIFORM1F(GLint location, GLfloat x)
{
    DEBUG_INFO("GL call: glUniform4f(%d, %f)\n", location, x);
    glUniform1f(location, x);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLUNIFORM4F(GLint location, GLfloat x, GLfloat y, GLfloat z, GLfloat w)
{
    DEBUG_INFO("GL call: glUniform4f(%d, %f, %f, %f, %f)\n", location, x, y, z, w);
    glUniform4f(location, x, y, z, w);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLCOMPRESSEDTEXIMAGE2D(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLint height, GLint border, GLsizei size, const GLvoid *data)
{
    DEBUG_NOTIFICATION("GL call: glCompressedTexImage2D(0x%x, %d, 0x%x, %d, %d, %d, %d, %p)\n",
               target, level, internalformat, width, height, border, size, data);
    GFXDEBUG_ADDCOMPTEXTUREDATA(size);
    glCompressedTexImage2D(target, level, internalformat, width, height, border, size, data);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLDRAWELEMENTS(GLenum mode, GLsizei count, GLenum type, const GLvoid *data)

{
    DEBUG_INFO("GL call: glDrawElements(%d, %d, %d, %p)\n", mode, count, type, data);
    GFXDEBUG_ADDTRIANGES(count/3);
    GFXDEBUG_ADDBATCHES(1);
    glDrawElements(mode, count, type, data);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLGENBUFFERS(GLsizei n, GLuint *buffers)
{
    DEBUG_NOTIFICATION("GL call: glGenBuffers(%d, %p)\n", n, buffers);
    glGenBuffers(n, buffers);
    FLUSH_GL_ERRORS();
}

static GLuint __cached_bindbuffer_array = -1;
static GLuint __cached_bindbuffer_elements = -1;
void GLWrapper::GLBINDBUFFER(GLenum target, GLuint buffer)
{
    GFXDEBUG_ADDBUFFERS(1);
    if (target == GL_ARRAY_BUFFER)
    {
        if (__cached_bindbuffer_array != buffer)
        {
            DEBUG_INFO("GL call: glBindBuffer(%d, %d)\n", target, buffer);
            glBindBuffer(target, buffer);
            __cached_bindbuffer_array = buffer;
            FLUSH_GL_ERRORS();
            return;
        }
        DEBUG_INFO("Cached GL call: glBindBuffer(%d, %d)\n", target, buffer);
    }
    else if (target == GL_ELEMENT_ARRAY_BUFFER)
    {
        if (__cached_bindbuffer_elements != buffer)
        {
            DEBUG_INFO("GL call: glBindBuffer(%d, %d)\n", target, buffer);
            glBindBuffer(target, buffer);
            __cached_bindbuffer_elements = buffer;
            FLUSH_GL_ERRORS();
            return;
        }
        DEBUG_INFO("Cached GL call: glBindBuffer(%d, %d)\n", target, buffer);
    }
}

void GLWrapper::GLBUFFERDATA(GLenum target, GLsizeiptr size, const GLvoid *data, GLenum usage)
{
    DEBUG_NOTIFICATION("GL call: glBufferData(%d, %d, %p, %d)\n", target, size, data, usage);
    GFXDEBUG_ADDGEOMETRYDATA(size);
    glBufferData(target, size, data, usage);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLDELETEBUFFERS(GLsizei n, const GLuint *buffers)
{
    DEBUG_NOTIFICATION("GL call: glDeleteBuffers(%d, %p)\n", n, buffers);
    glDeleteBuffers(n, buffers);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLDELETETEXTURES(GLsizei n, const GLuint *textures)
{
    DEBUG_NOTIFICATION("GL call: glDeleteTextures(%d, %p)\n", n, textures);
    glDeleteTextures(n, textures);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLGETSHADERIV(GLuint shader, GLenum pname, GLint *params)
{
    DEBUG_NOTIFICATION("GL call: glGetShaderiv(%d, %d, %p)\n", shader, pname, params);
    glGetShaderiv(shader, pname, params);
    FLUSH_GL_ERRORS();
}

GLuint GLWrapper::GLCREATESHADER(GLenum type)
{
    GLuint rc;
    rc = glCreateShader(type);
    DEBUG_NOTIFICATION("GL call: glCreateShader(%d) = %d\n", type, rc);
    FLUSH_GL_ERRORS();
    return rc;
}

void GLWrapper::GLGETSHADERINFOLOG(GLuint shader, GLsizei bufsize, GLsizei *length, GLchar *infolog)
{
    DEBUG_NOTIFICATION("GL call: glGetShaderInfoLog(%d, %d, %p, %p)\n", shader, bufsize, length, infolog);
    glGetShaderInfoLog(shader, bufsize, length, infolog);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLSHADERSOURCE(GLuint shader, GLsizei count, const GLchar **string, const GLint *length)
{
    DEBUG_NOTIFICATION("GL call: glShaderSource(%d, %d, %p, %p)\n", shader, count, string, length);
    glShaderSource(shader, count, string, length);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLCOMPILESHADER(GLuint shader)
{
    DEBUG_NOTIFICATION("GL call: glCompileShader(%d)\n", shader);
    glCompileShader(shader);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLDELETEPROGRAM(GLuint program)
{
    DEBUG_NOTIFICATION("GL call: glDeleteProgram(%d)\n", program);
    glDeleteProgram(program);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLDELETESHADER(GLuint shader)
{
    DEBUG_NOTIFICATION("GL call: glDeleteShader(%d)\n", shader);
    glDeleteShader(shader);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLENABLE(GLenum e)
{
    DEBUG_NOTIFICATION("GL call: glEnable(%d)\n", e);
    glEnable(e);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLDISABLE(GLenum e)
{
    DEBUG_NOTIFICATION("GL call: glDisable(%d)\n", e);
    glDisable(e);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLUNIFORMMATRIX4FV(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value)
{
    DEBUG_INFO("GL call: glUniformMatrix4fv(%d, %d, %d, %p)\n", location, count, transpose, value);
    glUniformMatrix4fv(location, count, transpose, value);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLBLENDFUNC(GLenum sfactor, GLenum dfactor)
{
    DEBUG_NOTIFICATION("GL call: glBlendFunc(%d, %d)\n", sfactor, dfactor);
    glBlendFunc(sfactor, dfactor);
    FLUSH_GL_ERRORS();
}

const GLubyte * GLWrapper::GLGETSTRING(GLenum e)
{
    const GLubyte *p;
    DEBUG_NOTIFICATION("GL call: glGetString(%d)\n", e);
    p = glGetString(e);
    FLUSH_GL_ERRORS();
    return p;
}

void GLWrapper::GLGETINTEGERV(GLenum name, GLint *param)
{
    DEBUG_NOTIFICATION("GL call: glGetIntegerv(%d, %p)\n", name, param);
    glGetIntegerv(name, param);
    FLUSH_GL_ERRORS();
}

void GLWrapper::GLGENERATEMIPMAP(GLenum target)
{
    DEBUG_NOTIFICATION("GL call: glGenerateMipmap(%d)\n", target);
    glGenerateMipmap(target);
    FLUSH_GL_ERRORS();
}

GLint GLWrapper::GLGETERROR(void)
{
    GLint error = glGetError();
#if defined(_DEBUG)
    if (error != GL_NO_ERROR)
        DEBUG_NOTIFICATION("GL call: glGetError() = %d\n", error);
#endif
    return error;
}

/******************************************************************************
 * EGL Wrappers
 */

#if defined(NC_EGL)

EGLBoolean GLWrapper::EGLSWAPBUFFERS(EGLDisplay egl_display, EGLSurface egl_surface)
{
    EGLBoolean b;
    b = eglSwapBuffers(egl_display, egl_surface);
    DEBUG_INFO("EGL call: eglSwapBuffers(%p, %p) = %d\n", egl_display, egl_surface, b);
    FLUSH_EGL_ERRORS();
    return b;
}

EGLDisplay GLWrapper::EGLGETDISPLAY(NativeDisplayType disp)
{
    EGLDisplay b;
    b = eglGetDisplay(disp);
    DEBUG_INFO("EGL call: eglGetDisplay(%d) = %d\n", disp, b);
    FLUSH_EGL_ERRORS();
    return b;
}

EGLBoolean GLWrapper::EGLINITIALIZE(EGLDisplay disp, EGLint *major, EGLint *minor)
{
    EGLBoolean b;
    b = eglInitialize(disp, major, minor);
    DEBUG_INFO("EGL call: eglInitialize(%d, %p, %p) = %d\n", disp, major, minor, b);
    FLUSH_EGL_ERRORS();
    return b;
}

EGLBoolean GLWrapper::EGLCHOOSECONFIG(EGLDisplay disp, EGLint const *attr, EGLConfig *c, EGLint cSize, EGLint *nConf)
{
    EGLBoolean b;
    b = eglChooseConfig(disp, attr, c, cSize, nConf);
    DEBUG_INFO("EGL call: eglChooseConfig(%d, %p, %p, %d, %p) = %d\n", disp, attr, c, cSize, nConf, b);
    FLUSH_EGL_ERRORS();
    return b;
}

EGLSurface GLWrapper::EGLCREATEWINDOWSURFACE(EGLDisplay disp, EGLConfig c, NativeWindowType nw, EGLint const *attr)
{
    EGLSurface b;
    b = eglCreateWindowSurface(disp, c, nw, attr);
    DEBUG_INFO("EGL call: eglCreateWindowSurface(%d, %d, %p, %p) = %d\n", disp, c, nw, attr, b);
    FLUSH_EGL_ERRORS();
    return b;
}

EGLContext GLWrapper::EGLCREATECONTEXT(EGLDisplay disp, EGLConfig c, EGLContext ctx, const EGLint *attr)
{
    EGLContext b;
    b = eglCreateContext(disp, c, ctx, attr);
    DEBUG_INFO("EGL call: eglCreateContext(%d, %d, %d, %p) = %d\n", disp, c, ctx, attr, b);
    FLUSH_EGL_ERRORS();
    return b;
}

EGLBoolean GLWrapper::EGLMAKECURRENT(EGLDisplay disp, EGLSurface s, EGLSurface r, EGLContext c)
{
    EGLBoolean b;
    b = eglMakeCurrent(disp, s, r, c);
    DEBUG_INFO("EGL call: eglMakeCurrent(%d, %d, %d, %d) = %d\n", disp, s, r, c, b);
    FLUSH_EGL_ERRORS();
    return b;
}

#endif

#if defined(NC_GLX)

#endif
