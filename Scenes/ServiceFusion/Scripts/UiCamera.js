// UiCamera.js - 3D UI Camera

// !ref: Scene1.txml
// !ref: Scene2.txml
// !ref: Scene3.txml
// !ref: Scene4.txml
// !ref ADD_SCENE_FILENAME_HERE

// Scene rotation
var sceneIndex = -1;
const scenes = ["Scene1.txml", "Scene2.txml", "Scene3.txml", "Scene4.txml"];
// Default content (Scene.txml) always present in the scene
var /*EntityMap*/ defaultContent = {};
// Current content of the active scene
var /*EntityList*/ currentContent = [];
// Screen resolution of the target device will be 1200x800
const cReferenceHeight = 800;
// Currently selected object, if any, as signaled by ObjectMove script
var selectedObject = null;

const cTouchInputOnly = false;

var cameraData =
{
    rotate :
    {
        sensitivity : 0.3
    },
    move :
    {
        //sensitivity : 30.0,
        sensitivity : 20.0,
        amount : new float3(0,0,0)
    },
    motion : new float3(0,0,0)
};

const cMoveZSpeed = 0.0007 // in Unity
const cMinTiltAngle = 110;
const cMaxTiltAngle = 170;
const cMinDistanceFromGround = 200 * 200; // squared distance
const cMaxDistanceFromGround = 700 * 700; // squared distance
var moving = false; // Is camera in moving state
var tilting = false; // Is camera in tilting state
var prevFrameMouseX = -1;
var prevFrameMouseY = -1;

// Entry point for the script
if (!framework.IsHeadless())
{
    engine.ImportExtension("qt.core");
    engine.ImportExtension("qt.gui");
    engine.IncludeFile("Log.js");
    engine.IncludeFile("MathUtils.js");
    engine.IncludeFile("Utils.js")
    
    ic = input.RegisterInputContextRaw("UiCamera", 102);
    ic.KeyEventReceived.connect(HandleKeyEvent);
    ic.MouseEventReceived.connect(HandleMouseEvent);
    ui.GraphicsView().DropEvent.connect(HandleDropEvent);

    // This is here to stop moving the uicamera if drag'n'drop widget is misplaced in the scene. Hax.
    ui.GraphicsView().DropEvent.connect(HandleDropEvent);

    frame.DelayedExecute(1).Triggered.connect(ApplyCamera);
    me.Action("ResetCamera").Triggered.connect(ResetCamera);
    me.Action("ObjectSelected").Triggered.connect(function(id) {
        selectedObject = scene.EntityById(parseInt(id));
    });
    me.Action("StopMovement").Triggered.connect(StopMovement);
}

function OnScriptDestroyed()
{
    if (!framework.IsExiting())
    {
        input.UnregisterInputContextRaw("UiCamera");
        ClearScene();
    }
}

function MoveToNextScene()
{
    if (++sceneIndex > scenes.length-1)
        sceneIndex = 0;
    SwitchScene();
}

function MoveToPreviousScene()
{
    if (--sceneIndex < 0)
        sceneIndex = scenes.length-1;
    SwitchScene();
}

function ClearScene()
{
    // TODO save old scene?
    for(i in currentContent)
    {
        currentContent[i].Exec(1, "Cleanup"); // Invoke special EA for screen script cleanup
        scene.RemoveEntity(currentContent[i].id);
    }
    currentContent = [];
}

function ResetScene()
{
    Log("Resetting scene.");
    for(i in defaultContent)
        defaultContent[i].Exec(1, "Reset");
    SwitchScene();
    ResetCamera();
}

function SwitchScene()
{
    if (sceneIndex < 0 || sceneIndex >= scenes.length)
    {
        LogE("SwitchScene: invalid scene index: " + sceneIndex);
        return;
    }

    ClearScene();

    Log("Loading scene " + sceneIndex + " " + scenes[sceneIndex]);

    currentContent = scene.LoadSceneXML(asset.GetAsset(scenes[sceneIndex]).DiskSource(), false, false, 0);
    // Comment out the code below and find-replace ADD_SCENE_FILENAME_HERE with the actual filename 
    // in order to enable the additional content not in this repo.
/*
    currentContent = currentContent.concat(scene.LoadSceneXML(asset.GetAsset("ADD_SCENE_FILENAME_HERE").DiskSource(), false, false, 0));
    scene.RemoveEntity(scene.EntityByName("oulu").id);
    var fogs = scene.EntitiesWithComponent("EC_Fog");
    for(i in fogs)
        scene.RemoveEntity(fogs[i].id);
*/
}

function ApplyCamera()
{
//    scene.EntityByName("FreeLookCamera").camera.nearPlane = 5.0;
//    scene.EntityByName("FreeLookCamera").soundlistener.active = false;
    me.camera.SetActive();
    me.soundlistener.active = true;
    ResetCamera();

    // Save default content
    defaultContent = scene.Entities();
    // Load the first scene.
    ++sceneIndex;
    SwitchScene();

    input.TouchBegin.connect(TouchUpdate);
    input.TouchUpdate.connect(TouchUpdate);
    input.TouchEnd.connect(OnTouchEnd);

    frame.Updated.connect(Update);
}

function TouchUpdate(e)
{
    var touches = e.touchPoints();
    var touchCount = touches.length;
    for(i in touches)
    {
        prevFrameMouseX = Math.round(touches[i].pos().x());
        prevFrameMouseY = Math.round(touches[i].pos().y());
        break; // for now, just use the pos from the first touch
    }

    if (selectedObject)
        return;

    TouchMove(touchCount, touches, e);
    TouchZoom(touchCount, touches, e);
    TouchResetScene(touchCount, touches, e);
    TouchChangeScene(touchCount, touches, e);
}

function OnTouchEnd(e)
{
    StopMovement();
    prevFrameMouseX = prevFrameMouseY = -1;
}

function ResetCamera()
{
    var resetTr = scene.EntityByName("UiCameraCameraSpawnPos").placeable.transform;
    me.placeable.transform = resetTr;
}

function HandleKeyEvent(e)
{
    if (e.eventType == 3 &&  e.HasCtrlModifier() && e.keyCode == Qt.Key_Left)
    {
        MoveToPreviousScene();
        e.Suppress();
    }
    if (e.eventType == 3 && e.HasCtrlModifier() && e.keyCode == Qt.Key_Right)
    {
        MoveToNextScene();
        e.Suppress();
    }
    if (e.HasCtrlModifier() && e.keyCode == Qt.Key_Tab)
        ToggleCamera();
    if (e.HasCtrlModifier() && e.keyCode == Qt.Key_R)
        ResetCamera();
    if (e.HasCtrlModifier() && e.HasAltModifier() && e.keyCode == Qt.Key_R)
        ResetScene();
}

function HandleMouseEvent(e)
{
    if (cTouchInputOnly)
        return;
    if (!me.camera.IsActive())
        return;
    if (selectedObject)
        return;
    switch(e.eventType)
    {
    case 1: // MouseMove
        if (moving)
        {
            if (input.ItemAtCoords(e.x, e.y))
                StopMovement(); // Mouse went on top of drop-down console or some other widget, abort movement.
            else
                HandleMove(e.relativeX, e.relativeY, 0);
        }
        else if (tilting)
        {
            var transform = me.placeable.transform;
            transform.rot.x -= cameraData.rotate.sensitivity * e.relativeY;
            var oldRotX = transform.rot.x;
            transform.rot.x = Clamp(transform.rot.x, cMinTiltAngle, cMaxTiltAngle);
            me.placeable.transform = transform;

            if (oldRotX > cMinTiltAngle && oldRotX < cMaxTiltAngle)
            {
                var d = e.relativeY * cMoveZSpeed * 30;
                var newPos = me.placeable.WorldPosition();
                newPos = newPos.Add(scene.UpVector().Mul(d));
                me.placeable.SetPosition(newPos);
            }
        }
        break;
    case 2: // MouseScroll
        if (moving)
        {
//            HandleMove(0, 0, e.relativeZ);
            var d = Clamp(e.relativeZ/30, -10, 10);
            Zoom(d);
        }
        break;
    case 3: // MousePressed
        if (e.button == 1)
        {
            var result = scene.ogre.Raycast(e.x, e.y);
            moving = !(result.entity && (IsObjectMovable(result.entity) || IsObjectFocusable(result.entity)));
        }
        else if (e.button == 2)
        {
            var result = scene.ogre.Raycast(e.x, e.y);
            tilting = !(result.entity && (IsObjectMovable(result.entity) || IsObjectFocusable(result.entity)));
            if (tilting)
            {
                var mousePos = input.MousePos();
                var relX = mousePos.x()/ui.GraphicsScene().width();
                var relY = mousePos.y()/ui.GraphicsScene().height();
                var ray = renderer.MainCameraComponent().GetMouseRay(relX, relY);
                var raycast = scene.ogre.Raycast(ray, -1);
                if (raycast.entity)
                    globePosition = raycast.pos;
            }
        }
        break;
    case 4: //MouseReleased
        StopMovement();
        break;
    default:
        break;
    }
}

function HandleDropEvent()
{
    // Stops the movement of uicamera if drag object is misplaced.
    frame.DelayedExecute(1.0/60).Triggered.connect(StopMovement);
}

function Zoom(d)
{
    var newPos = me.placeable.WorldPosition();
    var dir = me.placeable.Orientation().Mul(scene.ForwardVector()).Normalized();
 
    // Check that we stay within reasonable distance from the ground
    var r = scene.ogre.Raycast(new Ray(newPos, dir), 3);
    if (r.entity)
    {
        var distanceFromGround = r.pos.DistanceSq(newPos);
        if ((distanceFromGround > cMaxDistanceFromGround && d < 0) || 
            (distanceFromGround < cMinDistanceFromGround && d > 0))
        {
            return;
        }
    }

    newPos = newPos.Add(dir.Mul(d));
    me.placeable.SetPosition(newPos);
}

function ToggleCamera()
{
    // For camera switching to work, must have both the freeLookCamera & uiCamera in the scene
    var freeLookCameraEntity = scene.EntityByName("FreeLookCamera");
    if (freeLookCameraEntity == null || freeLookCameraEntity.camera == null || me == null)
    {
        LogE("ToggleCamera: Cameras not initialized properly.");
        return;
    }

    if (me.camera.IsActive())
    {
/*
        var trans = freeLookCameraEntity.placeable.transform;
        trans.pos = me.placeable.WorldPosition();
        trans.SetOrientation(me.placeable.WorldOrientation());

        // If there is roll in the rotation, adjust it away
        if (trans.rot.z > 170.0)
        {
            trans.rot.x -= 180.0;
            trans.rot.z = 0;
            trans.rot.y = -90.0 - (90.0 + trans.rot.y);
        }
        if (trans.rot.z < -170.0)
        {
            trans.rot.x += 180.0;
            trans.rot.z = 0;
            trans.rot.y = -90.0 - (90.0 + trans.rot.y);
        }

        freeLookCameraEntity.placeable.transform = trans;
*/
        freeLookCameraEntity.camera.SetActive();
        if (freeLookCameraEntity.soundlistener)
            freeLookCameraEntity.soundlistener.active = true;
    }
    else
    {
        me.camera.SetActive();
        if (freeLookCameraEntity.soundlistener)
            freeLookCameraEntity.soundlistener.active = false;
    }
}

var timer = 0;

function Update(frametime)
{
    if (!me.camera.IsActive())
        return;
    if (cameraData.move.amount.x == 0 && cameraData.move.amount.y == 0 && cameraData.move.amount.z == 0)
        return;

    cameraData.motion.x = cameraData.move.amount.x * cameraData.move.sensitivity * frametime;
    cameraData.motion.y = cameraData.move.amount.y * cameraData.move.sensitivity * frametime;
    cameraData.motion.z = cameraData.move.amount.z * cameraData.move.sensitivity * frametime;

    // Slow down the motion gradually
    timer += frametime;
    if (timer > 1)
        timer = 0;

    cameraData.move.amount.x = Lerp(cameraData.move.amount.x, 0, timer);
    cameraData.move.amount.y = Lerp(cameraData.move.amount.y, 0, timer);
    cameraData.move.amount.z = Lerp(cameraData.move.amount.z, 0, timer);

    // Note: do not take orientation into account, we want to move along the world axes
//    cameraData.motion = me.placeable.Orientation().Mul(cameraData.motion);
    me.placeable.SetPosition(me.placeable.Position().Add(cameraData.motion));
}

function HandleMove(deltaX, deltaY, deltaZ)
{
    if (!me.camera.IsActive())
        return;
    // forward/backward
    cameraData.move.amount.z = 2 * deltaY;
    // right/left
    cameraData.move.amount.x = 2 * deltaX;
    // up/down
    var scroll = deltaZ/30;
    cameraData.move.amount.y = Clamp(scroll, -5, 5);

    var movieDim = scene.GetEntityByName("MovieDim");
    if(movieDim)
        movieDim.Exec(1, "Move");
}

function StopMovement()
{
    moving = false;
    tilting = false;

    if (!me.camera.IsActive())
        return;
    cameraData.move.amount.z = 0;
    cameraData.move.amount.x = 0;
    cameraData.move.amount.y = 0;
/*
    if (param == "forward" && cameraData.move.amount.z == -1)
        cameraData.move.amount.z = 0;
    else if (param == "back" && cameraData.move.amount.z == 1)
        cameraData.move.amount.z = 0;
    else if (param == "right" && cameraData.move.amount.x == 1)
        cameraData.move.amount.x = 0;
    else if (param == "left" && cameraData.move.amount.x == -1)
        cameraData.move.amount.x = 0;
    else if (param == "up" && cameraData.move.amount.y == 1)
        cameraData.move.amount.y = 0;
    else if (param == "down" && cameraData.move.amount.y == -1)
        cameraData.move.amount.y = 0;
*/
}

var /*float*/ lastDistance = 0;
const /*float*/ cZoomSpeed = 0.9; // 0.02 in Unity;
const /*float*/ cTranslateSpeed = 0.012;

function TouchMove(touchCount, touches, e)
{
    if (touchCount == 1 && touches[0].state() == Qt.TouchPointMoved)
    {
        //if (input.ItemAtCoords(e.x, e.y))
        //    StopMovement(); // Mouse went on top of drop-down console or some other widget, abort movement.
        //else
        var delta = SubQPointF(touches[0].pos(), touches[0].lastPos());
        HandleMove(delta.x, delta.y, 0);
    }
    else if (touchCount == 2 && touches[0].state() == Qt.TouchPointMoved && 
        touches[1].state() == Qt.TouchPointMoved && input.IsKeyDown(Qt.Key_Control)) // TODO touchCount == 3
    {
        var delta = SubQPointF(touches[0].pos(), touches[0].lastPos());
        var t = me.placeable.transform;
        t.rot.x -= cameraData.rotate.sensitivity * delta.y;
        var oldRotX = t.rot.x;
        t.rot.x = Clamp(t.rot.x, cMinTiltAngle, cMaxTiltAngle);
        me.placeable.transform = t;

        if (oldRotX > cMinTiltAngle && oldRotX < cMaxTiltAngle)
        {
            var d = delta.y * cMoveZSpeed;
            var newPos = me.placeable.WorldPosition();
            newPos = newPos.Add(scene.UpVector().Mul(d));
            me.placeable.SetPosition(newPos);
        }
    }
}

function TouchZoom(touchCount, touches, e)
{
//    if (!allowRotate && touchCount == 0)
//        allowRotate = true;

    if (touchCount == 2 && !input.IsKeyDown(Qt.Key_Control)/* && !iconMove.IsSelected*/)
    {
        if (lastDistance == 0)
        {
            lastDistance = DistanceQPointF(touches[0].pos(), touches[1].pos());
            //Log("lastDistance " + lastDistance);
        }
        else if (touches[0].state() == Qt.TouchPointMoved && touches[1].state() == Qt.TouchPointMoved && lastDistance > 0)
        {
            var distance = DistanceQPointF(touches[0].pos(), touches[1].pos());
            var dotP = Math.abs(SubQPointF(touches[0].pos(), touches[1].pos()).Normalized().Dot(new float2(1,1).Normalized()));
            //Log("dotP " + dotP);
            //Log("Math.abs(1 - dotP) " + Math.abs(1 - dotP));
            if (/*Math.abs(distance - lastDistance) > 3f &&*/ Math.abs(1 - dotP) < 0.8/*0.2*/)
            {
                Zoom((distance - lastDistance) * (cReferenceHeight / ui.GraphicsScene().height()) * cZoomSpeed);
                lastDistance = distance;
            }
        }
    }
    else
        lastDistance = 0;
}

var /*QPointF*/ changeSceneTouchStart;
var /*float3*/ changeSceneStartPos;

function TouchChangeScene(touchCount, touches, e)
{
    if (touchCount == 2 /* TODO 3*/ && input.IsKeyDown(Qt.Key_Control))
    {
        for(i in touches)
        {
            if (touches[i].state() == Qt.TouchPointPressed)
            {
                changeSceneTouchStart = touches[i].pos();
//                changeSceneStartPos = transform.position;
                break;
            }
            else if (touches[i].state() == Qt.TouchPointReleased)
            {
                if (changeSceneTouchStart && Math.abs(touches[i].pos().x() - changeSceneTouchStart.x()) > ui.GraphicsScene().width() * 0.2/*0.12*/)
                {
                    if (touches[i].pos().x > changeSceneTouchStart.x())
                        MoveToPrevScene();
                    else
                        MoveToNextScene();

                    break;
                }
                else
                {
                    //transform.position = changeSceneStartPos;
                }
            }
            else if (touches[i].state() == Qt.TouchPointMoved)
            {
                //transform.position -= transform.right * touches[i].deltaPosition.x * 0.001f;
            }
        }
    }

    for(i in touches)
        if (touches[i].state() == Qt.TouchPointReleased && touchCount == 2 /* TODO touchCount == 3*/ && input.IsKeyDown(Qt.Key_Control))
        {
            //transform.position = changeSceneStartPos;
            changeSceneTouchStart = touches[i].pos();
        }
}

var sceneResetTouchStartTime = 0;
const cSceneResetTime = 3;

function TouchResetScene(touchCount, touches, e)
{
    if (touchCount == 2 /* TODO touchCount == 5 only */ && input.IsKeyDown(Qt.Key_Control) && input.IsKeyDown(Qt.Key_Shift))
    {
        for(i in touches)
        {
            if (touches[i].state() == Qt.TouchPointPressed)
            {
                sceneResetTouchStartTime = frame.WallClockTime();
                break;
            }
            else if (touches[i].state() == Qt.TouchPointReleased)
            {
                if (frame.WallClockTime() - sceneResetTouchStartTime >= cSceneResetTime)
                {
                    ResetScene();
                    break;
                }
            }
        }
    }
}
