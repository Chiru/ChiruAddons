
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef GLWrapper_H
#define GLWrapper_H

#include "Neocortex_GLHeaders.h"

class GLWrapper {
public:
    static GLWrapper * Instance();
    ~GLWrapper();

    void flushEGLErrors(void);

    // GL wrappers, for better handling of errors et al
    void   GLATTACHSHADER(GLuint shaderProgram, GLuint shader);
    void   GLBINDATTRIBLOCATION(GLuint shaderProgram, GLuint index, const GLchar *name);
    void   GLCLEARCOLOR(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha);
    GLuint GLCREATEPROGRAM(void);
    void   GLLINKPROGRAM(GLuint program);
    void   GLUSEPROGRAM(GLuint program);
    void   GLVIEWPORT(GLint x, GLint y, GLsizei width, GLsizei height);
    void   GLCLEAR(GLbitfield mask);
    void   GLVERTEXATTRIBPOINTER(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const GLvoid *data);
    void   GLENABLEVERTEXATTRIBARRAY(GLuint index);
    void   GLDISABLEVERTEXATTRIBARRAY(GLuint index);
    void   GLDRAWARRAYS(GLenum mode, GLint first, GLsizei size);
    void   GLGENTEXTURES(GLsizei size, GLuint *ptr);
    void   GLBINDTEXTURE(GLenum target, GLuint id);
    void   GLPIXELSTOREI(GLenum type, GLint align);
    void   GLTEXIMAGE2D(GLenum target, GLint level, GLint inernalformat, GLsizei width,
                                       GLsizei height, GLint border, GLenum format, GLenum type, const void *pixels);
    void   GLTEXPARAMETERI(GLenum target, GLenum pname, GLint param);
    GLint  GLGETATTRIBLOCATION(GLuint program, const GLchar *name);
    GLint  GLGETUNIFORMLOCATION(GLuint program, const GLchar *name);
    void   GLACTIVETEXTURE(GLenum texture);
    void   GLUNIFORM1I(GLint location, GLint x);
    void   GLUNIFORM1F(GLint location, GLfloat x);
    void   GLUNIFORM4F(GLint location, GLfloat x, GLfloat y, GLfloat z, GLfloat w);
    void   GLCOMPRESSEDTEXIMAGE2D(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLint height, GLint border,
                                       GLsizei size, const GLvoid *data);
    void   GLDRAWELEMENTS(GLenum mode, GLsizei count, GLenum type, const GLvoid *data);
    void   GLGENBUFFERS(GLsizei n, GLuint *buffers);
    void   GLBINDBUFFER(GLenum target, GLuint buffer);
    void   GLBUFFERDATA(GLenum target, GLsizeiptr size, const GLvoid *data, GLenum usage);
    void   GLDELETEBUFFERS(GLsizei n, const GLuint *buffers);
    void   GLDELETETEXTURES(GLsizei n, const GLuint *textures);

    void   GLGETSHADERIV(GLuint shader, GLenum pname, GLint *params);
    GLuint GLCREATESHADER(GLenum type);
    void   GLGETSHADERINFOLOG(GLuint shader, GLsizei bufsize, GLsizei *length, GLchar *infolog);
    void   GLSHADERSOURCE(GLuint shader, GLsizei count, const GLchar **string, const GLint *length);
    void   GLCOMPILESHADER(GLuint shader);
    void   GLDELETEPROGRAM(GLuint program);
    void   GLDELETESHADER(GLuint shader);
    void   GLENABLE(GLenum e);
    void   GLDISABLE(GLenum e);
    void   GLUNIFORMMATRIX4FV(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
    void   GLBLENDFUNC(GLenum sfactor, GLenum dfactor);

    const GLubyte * GLGETSTRING(GLenum e);
    void   GLGETINTEGERV(GLenum name, GLint *param);

    void  GLGENERATEMIPMAP(GLenum target);

    GLint GLGETERROR(void);

#if defined(NC_EGL)
    // EGL Wrappers:
    EGLBoolean EGLSWAPBUFFERS(EGLDisplay disp, EGLSurface egl_surface);
    EGLDisplay EGLGETDISPLAY(NativeDisplayType disp);
    EGLBoolean EGLINITIALIZE(EGLDisplay disp, EGLint *major, EGLint *minor);
    EGLBoolean EGLCHOOSECONFIG(EGLDisplay disp, EGLint const *attr, EGLConfig *c, EGLint cSize, EGLint *nConf);
    EGLSurface EGLCREATEWINDOWSURFACE(EGLDisplay disp, EGLConfig c, NativeWindowType nw, EGLint const *attr);
    EGLContext EGLCREATECONTEXT(EGLDisplay disp, EGLConfig c, EGLContext ctx, EGLint const *attr);
    EGLBoolean EGLMAKECURRENT(EGLDisplay disp, EGLSurface s, EGLSurface r, EGLContext c);
#endif

protected:

private:
    GLWrapper();
    GLWrapper(GLWrapper const &) {}
    static GLWrapper *p_Instance;                    // Single instance placeholder

    void flushGLErrors(void);

    // Error counters
    unsigned int GLerrors;
    unsigned int EGLerrors;
};

#endif // GLWrapper_H
