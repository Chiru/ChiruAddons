
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "NeocortexDisplay.h"
#include "DebugLog.h"
#include "GLWrapper.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <sys/stat.h>


#if defined(NC_EGL)
const EGLint attr[] = {       // some attributes to set up our egl-interface
   EGL_BUFFER_SIZE,     16,
   EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
   //EGL_ALPHA_SIZE,      8,
   EGL_DEPTH_SIZE,      8,
   EGL_NONE
};

const EGLint ctxattr[] = {
   EGL_CONTEXT_CLIENT_VERSION, 2,
   EGL_NONE
};
#elif defined(NC_GLX)
int singleBufferAttributess[] = {
    GLX_DRAWABLE_TYPE, GLX_WINDOW_BIT,
    GLX_RENDER_TYPE,   GLX_RGBA_BIT,
    GLX_RED_SIZE,      1,   /* Request a single buffered color buffer */
    GLX_GREEN_SIZE,    1,   /* with the maximum number of color bits  */
    GLX_BLUE_SIZE,     1,   /* for each component                     */
    None
};

int doubleBufferAttributes[] = {
    GLX_DRAWABLE_TYPE, GLX_WINDOW_BIT,
    GLX_RENDER_TYPE,   GLX_RGBA_BIT,
    GLX_DOUBLEBUFFER,  True,  /* Request a double-buffered color buffer with */
    GLX_RED_SIZE,      1,     /* the maximum number of bits per component    */
    GLX_GREEN_SIZE,    1,
    GLX_BLUE_SIZE,     1,
    None
};
#elif defined(NC_WIN32)
#endif


NeocortexDisplay::NeocortexDisplay() :
#if defined(NC_EGL)
    egl_context(0),
    egl_display(0),
    egl_surface(0),
    win(0),
    x_display(NULL)
#elif defined(NC_GLX)
    glx_context(0),
    glx_window(0),
    win(0),
    x_display(NULL)
#elif defined(NC_WIN32)
#endif
{
}


NeocortexDisplay::~NeocortexDisplay()
{
    destroyDisplay();
}


/******************************************************************************
 * Display and context creation helpers
 */

#if defined(NC_GLX)
//static Bool WaitForNotify( Display *dpy, XEvent *event, XPointer arg ) {
//    return (event->type == MapNotify) && (event->xmap.window == (Window) arg);
//}
#endif

int NeocortexDisplay::createDisplay(int width, int height, bool fullscreen)
{
    w_width = width;
    w_height = height;
    w_fullscreen = fullscreen;

#if defined(NC_EGL)
   Window root;
   XSetWindowAttributes swa;
   XSetWindowAttributes  xattr;
   Atom wm_state, a_fullscreen;
   XWMHints hints;
   XEvent xev;
   EGLConfig ecfg;
   EGLint num_config;

   /*
    * X11 native display initialization
    */

   DEBUG_INFO("Connecting to X server\n");
   x_display = XOpenDisplay(NULL);
   if ( x_display == NULL )
   {
       DEBUG_FATAL("Error: Unable to connect to X Server\n");
       return -1;
   }

   DEBUG_INFO("Querying X root window\n");
   root = DefaultRootWindow(x_display);

   DEBUG_INFO("Creating X11 window\n");
   swa.event_mask  =  ExposureMask | PointerMotionMask | KeyPressMask | ButtonPressMask | ButtonReleaseMask | ButtonMotionMask;
   win = XCreateWindow(
              x_display, root,
              0, 0, width, height, 0,
              CopyFromParent, InputOutput,
              CopyFromParent, CWEventMask,
              &swa );

   DEBUG_INFO("Updating window attributes\n");
   xattr.override_redirect = false;
   XChangeWindowAttributes ( x_display, win, CWOverrideRedirect, &xattr );

   DEBUG_INFO("Setting Window manager hints\n");
   hints.input = true;
   hints.flags = InputHint;
   XSetWMHints(x_display, win, &hints);

   // make the window visible on the screen
   DEBUG_INFO("Making window visible\n");
   XMapWindow (x_display, win);
   XStoreName (x_display, win, "Neocortex 3D Engine");

   // get identifiers for the provided atom name strings
   wm_state = XInternAtom (x_display, "_NET_WM_STATE", false);
   a_fullscreen = false;
   if (w_fullscreen == true)
       a_fullscreen = XInternAtom (x_display, "_NET_WM_STATE_FULLSCREEN", w_fullscreen);

   DEBUG_INFO("Updating window event masks\n");
   memset ( &xev, 0, sizeof(xev) );
   xev.type                 = ClientMessage;
   xev.xclient.window       = win;
   xev.xclient.message_type = wm_state;
   xev.xclient.format       = 32;
   xev.xclient.data.l[0]    = 1;
   xev.xclient.data.l[1]    = a_fullscreen;
   XSendEvent (
      x_display,
      DefaultRootWindow ( x_display ),
      false,
      SubstructureNotifyMask,
      &xev );

   DEBUG_NOTIFICATION("X11 native display init done!\n");

   /*
    * Now that the native window is up, we shall initialize EGL
    */

   DEBUG_INFO("EGL: eglGetDisplay()\n");
   egl_display  =  eglGetDisplay( (EGLNativeDisplayType) x_display );
   if ( egl_display == EGL_NO_DISPLAY ) {
       DEBUG_FATAL("EGL: eglGetDisplay() failed!\n");
       return -1;
   }

   DEBUG_INFO("EGL: eglInitialize()\n");
   if ( !eglInitialize( egl_display, NULL, NULL ) ) {
       DEBUG_FATAL("EGL: eglInitialize() failed!\n");
       return -1;
   }

   DEBUG_INFO("EGL: eglChooseConfig()\n");
   if ( !eglChooseConfig( egl_display, attr, &ecfg, 1, &num_config ) ) {
       DEBUG_FATAL("EGL: eglChooseConfig() failed!\n");
       return -1;
   }

   DEBUG_INFO("EGL: EGL configs available?\n");
   if ( num_config == 0 ) {
       DEBUG_FATAL("EGL: eglGetDisplay() no configs found!\n");
       return -1;
   }

   DEBUG_INFO("EGL: eglCreateWindowSurface()\n");
   egl_surface = eglCreateWindowSurface ( egl_display, ecfg, win, NULL );
   if ( egl_surface == EGL_NO_SURFACE ) {
       DEBUG_FATAL("EGL: eglCreateWindowSurface() failed!\n");
       return -1;
   }

   DEBUG_WARNING("EGL: eglCreateContext()\n");
   egl_context = eglCreateContext ( egl_display, ecfg, EGL_NO_CONTEXT, ctxattr );
   if ( egl_context == EGL_NO_CONTEXT ) {
       DEBUG_FATAL("EGL: eglCreateContext() failed!\n");
       return -1;
   }

   DEBUG_INFO("EGL: eglMakeCurrent()\n");
   eglMakeCurrent( egl_display, egl_surface, egl_surface, egl_context );

   DEBUG_NOTIFICATION("EGL initialization completed!\n");
   return 0;

#elif defined(NC_GLX)

    //XEvent                event;
    XVisualInfo          *vInfo;
    XSetWindowAttributes  swa;
    GLXFBConfig          *fbConfigs;
    int                   swaMask;
    int                   numReturned;

    /* Open a connection to the X server */
    DEBUG_INFO("GLX: Opening X display\n");
    x_display = XOpenDisplay( NULL );
    if ( x_display == NULL ) {
        DEBUG_FATAL( "Unable to open a connection to the X server\n" );
        return -1;
    }

    /* Request a suitable framebuffer configuration - try for a double
    ** buffered configuration first */
    DEBUG_INFO("GLX: Checking available GLX configs\n");
    fbConfigs = glXChooseFBConfig( x_display, DefaultScreen(x_display),
                                   doubleBufferAttributes, &numReturned );

    if ( fbConfigs == NULL ) {  /* no double buffered configs available */
        DEBUG_INFO("GLX: Check failed, rechecking with singlebuffer flags\n");
        fbConfigs = glXChooseFBConfig( x_display, DefaultScreen(x_display),
                                     singleBufferAttributess, &numReturned );
        if (fbConfigs == NULL)
        {
            DEBUG_INFO("GLX: Failed again, no configs found. Aorting!\n");
            return -1;
        }
    }

    /* Create an X colormap and window with a visual matching the first
    ** returned framebuffer config */
    DEBUG_INFO("GLX: get visual\n");
    vInfo = glXGetVisualFromFBConfig( x_display, fbConfigs[0] );
    if (vInfo == NULL)
    {
        DEBUG_INFO("GLX: Unable to get visual\n");
        return -1;
    }

    swa.border_pixel = 0;
    swa.event_mask = ExposureMask | PointerMotionMask | KeyPressMask | ButtonPressMask | ButtonReleaseMask | ButtonMotionMask;
    DEBUG_INFO("GLX: Create colormap\n");
    swa.colormap = XCreateColormap( x_display, RootWindow(x_display, vInfo->screen),
                                    vInfo->visual, AllocNone );

    swaMask = CWBorderPixel | CWColormap | CWEventMask;

    DEBUG_INFO("GLX: create X window\n");
    win = XCreateWindow( x_display, RootWindow(x_display, vInfo->screen), 0, 0, width, height,
                               0, vInfo->depth, InputOutput, vInfo->visual,
                               swaMask, &swa );

    /* Create a GLX context for OpenGL rendering */
    DEBUG_INFO("GLX: create new GLX context\n");
    glx_context = glXCreateNewContext(x_display, fbConfigs[0], GLX_RGBA_TYPE,
                                      NULL, True);

    /* Create a GLX window to associate the frame buffer configuration
    ** with the created X window */
    DEBUG_INFO("GLX: create new GLX window\n");
    glx_window = glXCreateWindow( x_display, fbConfigs[0], win, NULL );

    /* Map the window to the screen, and wait for it to appear */
    DEBUG_INFO("GLX: Map X11 window\n");
    XMapWindow( x_display, win );
    //XIfEvent( x_display, &event, WaitForNotify, (XPointer) win );

    /* Bind the GLX context to the Window */
    DEBUG_INFO("GLX: Making GLX context current\n");
    glXMakeContextCurrent( x_display, glx_window, glx_window, glx_context );

    if (GLEW_OK != glewInit())
    {
        DEBUG_FATAL("GLX: GLEW init failed\n");
        return -1;
    }

    DEBUG_INFO("GLX: X11/GLX window creation ok\n");
    return 0;

#elif defined(NC_WIN32)

    {
       WNDCLASS wndclass = {0};
       DWORD    wStyle   = 0;
       RECT     windowRect;
       HINSTANCE hInstance = GetModuleHandle(NULL);

       wndclass.style         = CS_OWNDC;
       wndclass.lpfnWndProc   = (WNDPROC)EGLX11Display::ESWindowProc;
       wndclass.hInstance     = hInstance;
       wndclass.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
       wndclass.lpszClassName = "opengles2.0";

       if (!RegisterClass (&wndclass) )
          return FALSE;

       wStyle = WS_VISIBLE | WS_POPUP | WS_BORDER | WS_SYSMENU | WS_CAPTION;

       // Adjust the window rectangle so that the client area has
       // the correct number of pixels
       windowRect.left = 0;
       windowRect.top = 0;
       windowRect.right = w_width;
       windowRect.bottom = w_height;
       AdjustWindowRect ( &windowRect, wStyle, FALSE );

       hWnd = CreateWindow(
                             "opengles2.0",
                             "Neocortex",
                             wStyle,
                             0,
                             0,
                             windowRect.right - windowRect.left,
                             windowRect.bottom - windowRect.top,
                             NULL,
                             NULL,
                             hInstance,
                             NULL);

       // Set the ESContext* to the GWL_USERDATA so that it is available to the
       // ESWindowProc
       //SetWindowLongPtr (  hWnd, GWL_USERDATA, (LONG) (LONG_PTR) esContext );

       if ( hWnd == NULL )
       {
           DEBUG_FATAL("Unable to open a window\n");
           return -1;
       }
       ShowWindow(hWnd, TRUE);
    }

    {
       EGLint numConfigs;
       EGLint majorVersion;
       EGLint minorVersion;
       //EGLDisplay display;
       //EGLContext context;
       //EGLSurface surface;
       EGLConfig config;
       //EGLint contextAttribs[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE, EGL_NONE };

       // Get Display
       egl_display = eglGetDisplay(GetDC(hWnd));
       if ( egl_display == EGL_NO_DISPLAY )
       {
            DEBUG_FATAL("Unable to get EGL display\n");
            return -1;
       }

       // Initialize EGL
       if ( !eglInitialize(egl_display, &majorVersion, &minorVersion) )
       {
           DEBUG_FATAL("Unable to initialize EGL\n");
           return -1;
       }

       // Get configs
       if ( !eglGetConfigs(egl_display, NULL, 0, &numConfigs) )
       {
           DEBUG_FATAL("Unable to get EGL configs\n");
           return -1;
       }

       // Choose config
       if ( !eglChooseConfig(egl_display, attr, &config, 1, &numConfigs) )
       {
           DEBUG_FATAL("Unable to choose EGL config\n");
           return -1;
       }

       // Create a surface
       egl_surface = eglCreateWindowSurface(egl_display, config, (EGLNativeWindowType)hWnd, NULL);
       if ( egl_surface == EGL_NO_SURFACE )
       {
           DEBUG_FATAL("Unable to create Window Surface\n");
           return -1;
       }

       // Create a GL context
       egl_context = eglCreateContext(egl_display, config, EGL_NO_CONTEXT, ctxattr );
       if ( egl_context == EGL_NO_CONTEXT )
       {
           DEBUG_FATAL("Unable to create EGL context\n");
           return -1;
       }

       // Make the context current
       if ( !eglMakeCurrent(egl_display, egl_surface, egl_surface, egl_context) )
       {
           DEBUG_FATAL("EGL MakeCurrent failed\n");
           return -1;
       }
    }
    return 0;
#endif
}

int NeocortexDisplay::destroyDisplay(void)
{
#if defined(NC_EGL)
    if (egl_display != 0)
    {
        DEBUG_INFO("Starting EGL destruction\n");
        if (egl_context != 0)
            eglDestroyContext (egl_display, egl_context);
        if (egl_surface != 0)
            eglDestroySurface (egl_display, egl_surface);
        eglTerminate (egl_display);
        DEBUG_INFO("EGL Destruction done\n");
    }
    if (x_display != NULL)
    {
        DEBUG_INFO("Starting X11 destruction\n");
        if (win != 0)
            XDestroyWindow(x_display, win);
        XCloseDisplay(x_display);
        DEBUG_INFO("X11 Destruction done\n");
    }
    win = 0;
    x_display = NULL;
    egl_display = 0;
    egl_context = 0;
    egl_surface = 0;
    return 0;
#elif defined(NC_GLX)
    // GLXwindow...
    // GLXcontext...
    if (x_display != NULL)
    {
        DEBUG_INFO("Starting X11 destruction\n");
        if (win != 0)
            XDestroyWindow(x_display, win);
        XCloseDisplay(x_display);
        DEBUG_INFO("X11 Destruction done\n");
    }
    win = 0;
    x_display = NULL;
    glx_context = 0;
    glx_window = 0;
    return 0;
#elif defined(NC_WIN32)
    return 0;
#endif
}

int NeocortexDisplay::swapBuffers(void)
{
#if defined(NC_EGL)
    GLWrapper::Instance()->EGLSWAPBUFFERS(egl_display, egl_surface);
    return 0;
#elif defined(NC_GLX)
    glXSwapBuffers( x_display, glx_window );
    return 0;
#elif defined(NC_WIN32)
    return 0;
#endif
}

int NeocortexDisplay::eventPump(EGLX11EVENT *event)
{
#if defined(NC_EGL) || defined(NC_GLX)
    XEvent xev;
    KeySym key;
    char text;

    DEBUG_NOTIFICATION("Entering X11 event pump\n");
    event->event = EVENT_NONE;

    // Pump all messages from X server. Keypresses are directed to keyfunc (if defined)
    if ( XPending ( x_display ) )
    {
        XNextEvent( x_display, &xev );
        DEBUG_NOTIFICATION("X11 Event type: %d\n", xev.type);
        switch(xev.type)
        {
        case KeyPress:
            if (XLookupString(&xev.xkey,&text,1,&key,0)==1)
            {
                event->event = EVENT_KEYEVENT;
                event->u.keyEvent.key = text;
                event->u.keyEvent.a   = 0;
                event->u.keyEvent.b   = 0;
            }
            break;
        case ButtonPress:
            event->event = EVENT_MOUSEBUTTONEVENT;
            event->u.mouseButtonEvent.button = xev.xbutton.button;
            event->u.mouseButtonEvent.status = true;
            event->u.mouseButtonEvent.x = xev.xbutton.x;
            event->u.mouseButtonEvent.y = xev.xbutton.y;
            break;
        case ButtonRelease:
            event->event = EVENT_MOUSEBUTTONEVENT;
            event->u.mouseButtonEvent.button = xev.xbutton.button;
            event->u.mouseButtonEvent.status = false;
            event->u.mouseButtonEvent.x = xev.xbutton.x;
            event->u.mouseButtonEvent.y = xev.xbutton.y;
            break;
        case MotionNotify:
            event->event = EVENT_MOUSEMOTIONEVENT;
            event->u.mouseMotionEvent.x = xev.xmotion.x;
            event->u.mouseMotionEvent.y = xev.xmotion.y;
            break;
        }
        // X11 terminate event will override all events, hence is treated last here:
        if ( xev.type == DestroyNotify )
        {
            event->event = EVENT_QUIT;
        }
    }
    else DEBUG_NOTIFICATION("No more messages in the X11 event pump\n");

    // Return value is for easier pump control for the caller
    if (event->event == EVENT_NONE) return 0;
    return 1;

#elif defined(NC_WIN32)

    MSG msg = { 0 };
    DWORD lastTime = GetTickCount();

    while (!done)
    {
       int gotMsg = (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) != 0);
       DWORD curTime = GetTickCount();
       float deltaTime = (float)( curTime - lastTime ) / 1000.0f;
       lastTime = curTime;

       if ( gotMsg )
       {
          if (msg.message == WM_QUIT)
          {
              event->event = EVENT_QUIT;
          }
          else
          {
              TranslateMessage(&msg);
              DispatchMessage(&msg);
          }
       }
       else
          SendMessage( hWnd, WM_PAINT, 0, 0 );

       // Call update function if registered
       //if ( esContext->updateFunc != NULL )
       //   esContext->updateFunc ( esContext, deltaTime );
    }
#endif
}
