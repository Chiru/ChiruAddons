Getting Started
===============
- Scene.txml is the "entry point" of the application.
- Use "--config ServiceFusionPlugins.xml" when running the application.
- F.ex:
  cd <TundraBin>
  Tundra --config ../src/ChiruAddons/ServiceFusionPlugins.xml --file ../src/ChiruAddons/Scenes/ServiceFusion/Scene.txml

Controls
========
- Mouse + keyboard
 * LMB drag terrain - camera movement
 * RMB + drag terrain - camera tilting
 * Mouse scroll, zoom/depth movement
 * Ctrl + Left arrow/Right arrow - Switch scene
 * 1 + LMB drag object - Move object
 * Ctrl + 1 + LMB drag object - Zoom object
 * 2 + RMB drag object - Rotate object
 * Ctrl + R - Reset camera's transform
 * Ctrl + E - Reset selected object's transform
- Touch input TODO
 * not implemented yet
 * Scene
  - Move to next/prev. scene - 3 finger x-axis sweep
  - Move in scene - 1 finger drag
  - Tilt scene - 2 finger y-axis drag
  - "Zoom" scene (depth movement) - 2 finger pinching
 * Object
  - Object selection - long press
  - moving object
   * object rotation (yaw, pitch) - select object, second finger y-axis drag
   * object roll - select object, rotate second finger around object
   * "Zoom" object (depth movement) - select object, one finder bottom-right corner of the screen, second finger y-axis drag
 * Scene reset - 5 finger long touch