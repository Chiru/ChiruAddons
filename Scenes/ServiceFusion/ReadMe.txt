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
 * TODO Mouse scroll, zoom/depth movement
 * Ctrl + Left arrow/Right arrow - Switch scene
 * 1 + LMB drag object - Move object
 * Ctrl + 1 + LMB drag object - Zoom object
 * 2 + RMB drag object - Rotate object
 * Ctrl + R - Reset camera's transform
 * Ctrl + E - Reset selected object's transform
- Touch input TODO
 * not implemented yet
 * Scene
  - Move to next/prev. scene
  - Move in scene
  - Tilt scene
  - "Zoom" scene (depth movement)
 * Object
  - Object selection
  - moving object
   * object rotation (yaw, pitch)
   * object roll
   * "Zoom" object (depth movement)
 * Scene reset