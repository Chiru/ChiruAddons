
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef NeocortexDisplay_H
#define NeocortexDisplay_H

#include <iostream>
#include <string.h>
#include <stdio.h>

#include "Neocortex_GLHeaders.h"
#include "GLWrapper.h"

typedef enum {
    EVENT_NONE = 0,
    EVENT_KEYEVENT,
    EVENT_MOUSEBUTTONEVENT,
    EVENT_MOUSEMOTIONEVENT,
    // Termination
    EVENT_QUIT
} EVENT_TYPE;

typedef struct _eglx11event {
    EVENT_TYPE event;
    union {
        struct _keyevent {
            unsigned char key;
            int a;
            int b;
        } keyEvent;
        struct _mousebuttonevent {
            bool status;
            unsigned int button;
            unsigned int x;
            unsigned int y;
        } mouseButtonEvent;
        struct _mousemotionevent {
            int x;
            int y;
        } mouseMotionEvent;
    } u;
} EGLX11EVENT;


/// Class definition
class NeocortexDisplay
{
public:
    NeocortexDisplay();
    ~NeocortexDisplay();

    /// Benchmark class helper methods for EGL context handling
    int createDisplay(int width, int height, bool fullscreen);
    int destroyDisplay(void);
    int swapBuffers(void);

    int eventPump(EGLX11EVENT *event);

    int getDisplayWidth(void)  { return w_width; }
    int getDisplayHeight(void) { return w_height; }

private:
    // Logical render window properties
    unsigned int w_width;
    unsigned int w_height;
    bool w_fullscreen;

#if defined(NC_EGL)
    // Variables EGL
    EGLContext  egl_context;
    EGLDisplay  egl_display;
    EGLSurface  egl_surface;
    // Variables X11
    Window      win;
    Display     *x_display;
#elif defined(NC_GLX)
    GLXContext  glx_context;
    GLXWindow   glx_window;
    Window      win;
    Display     *x_display;
#elif defined(NC_WIN32)
    EGLNativeWindowType hWnd;
    LRESULT WINAPI ESWindowProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
#endif
};

#endif // NeocortexDisplay_H
