Service Fusion Tundra Application
*********************************

Getting Started
===============
- For now at least, ServiceFusion application is a standalone application with no Tundra client-server functionality whatsoever.
- IMPORTANT: Due to QtNetwork/Webkit regression issues introduced in Qt 4.8, use Qt version 4.7.x, for now
- If desiring to use touch input on Ubuntu, the Tundra command-line switch --ogreCaptureTopWindow is most likely needed.
  Also, detaching any external monitor and mouse devices is most likely required.
- Scene.txml is the "entry point" of the application.
- Use "--config ServiceFusionPlugins.xml" when running the application, f.ex.
  cd <Tundra>/bin
  Tundra --config ../src/ChiruAddons/ServiceFusionPlugins.xml --file ../src/ChiruAddons/Scenes/ServiceFusion/Scene.txml

Controls
========
- Mouse + keyboard
 * LMB drag terrain - camera movement
 * RMB + drag terrain - camera tilting
 * LMB + mouse scroll - zoom/depth movement
 * Ctrl + Left arrow/Right arrow - Switch scene
 * 1 + LMB drag object - Move object
 * Ctrl + 1 + LMB drag object - Zoom object
 * 2 + RMB drag object - Rotate object
 * Ctrl + R - Reset camera's transform
 * Ctrl + E - Reset selected object's transform
 * Ctrl + Alt + R - Reset scene
- Touch input
 * Scene
  - Move to next/prev. scene - 3 finger x-axis sweep
  - Move in scene - 1 finger drag
  - Tilt scene - 3 finger y-axis drag
  - "Zoom" scene (depth movement) - 2 finger pinching
 * Object
  - Object selection - long press
  - moving object
   * object rotation (yaw, pitch) - select object, second finger y-axis drag
   * object roll - select object, rotate second finger around object
   * "Zoom" object (depth movement) - select object, hold one finger at bottom-left corner of the screen, second finger y-axis drag
 * Scene reset - 5 finger long touch
