
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#if defined(NC_EGL)

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <EGL/egl.h>

#elif defined(NC_GLX)

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glext.h>

#elif defined(NC_WIN32)

#ifndef WIN32_MEAN_AND_LEAN
#define WIN32_MEAN_AND_LEAN 1
#endif
#include "windows.h"
#include "GL/gl.h"

#else

#error "No GL target flag defined. Must be one of NC_EGL, NC_GLX, NC_WIN32"

#endif
