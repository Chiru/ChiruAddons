// UiCamera.js - 3D UI Camera

// !ref: Scene1.txml
// !ref: Scene2.txml
// !ref: Scene3.txml

var sceneIndex = -1;
// Scene rotation
const scenes = ["Scene1.txml", "Scene2.txml", "Scene3.txml"];
var currentContent = [];

var cameraData =
{
    connected : false,
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

var lastTouchTimestamp = frame.WallClockTime();
const cMoveZSpeed = 0.03;//0.005; // was 0.0007 in Unity
const minTiltAngle = 110;
const maxTiltAngle = 170;
const minDistanceFromGround = 50;
const maxDistanceFromGround = 400;
var moving = false; // Is camera in moving state
var tilting = false; // Is camera in tilting state
var prevFrameMouseX = -1;
var prevFrameMouseY = -1;
var useTouchInput = false;

// Entry point for the script
if (!framework.IsHeadless())
{
    engine.ImportExtension("qt.core");
    engine.ImportExtension("qt.gui");
    engine.IncludeFile("Log.js");
    engine.IncludeFile("MathUtils.js");

    ic = input.RegisterInputContextRaw("UiCamera", 102);
    ic.KeyEventReceived.connect(HandleKeyEvent);
    ic.MouseEventReceived.connect(HandleMouseEvent);
    ic.GestureEventReceived(HandleGestureEvent);

    frame.DelayedExecute(1).Triggered.connect(ApplyCamera);
    me.Action("ResetCamera").Triggered.connect(ResetCamera);
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
    // TODO Reset camera etc?
}

function ApplyCamera()
{
//    scene.EntityByName("FreeLookCamera").camera.nearPlane = 5.0;
//    scene.EntityByName("FreeLookCamera").soundlistener.active = false;
    me.camera.SetActive();
    me.soundlistener.active = true;
    ResetCamera();

    // Load the first scene.
    ++sceneIndex;
    SwitchScene();

    input.TouchBegin.connect(OnTouchBegin);
    input.TouchUpdate.connect(OnTouchUpdate);
    input.TouchEnd.connect(OnTouchEnd);
//    Log("input.IsGesturesEnabled() " + input.IsGesturesEnabled());
    frame.Updated.connect(Update);
}

function OnTouchBegin(e)
{
    useTouchInput = true;
//    Log("UiCamera OnTouchBegin " + e.touchPoints().length);
    lastTouchTimestamp = frame.WallClockTime();
    var touches = e.touchPoints();
    for(i in touches)
    {
        prevFrameMouseX = Math.round(touches[i].pos().x());
        prevFrameMouseY = Math.round(touches[i].pos().y());
        break; // for now, just use the pos from the first touch
    }
}

var sceneResetTimer = 0;
var touchDeltaX, touchDeltaY;

function OnTouchUpdate(e)
{
//    Log("UiCamera OnTouchUpdate " + e.touchPoints().length);
    lastTouchTimestamp = frame.WallClockTime();
    var touches = e.touchPoints();
    var numFingers = touches.length;
/*
    if (numFingers == 5)
    {
        sceneResetTimer += (frame.WallClockTime() - lastTouchTimestamp);
        Log(sceneResetTimer);
        if (sceneResetTimer >= 5)
        {
            ResetScene();
            sceneResetTimer = 0;
        }
    }
    else
        sceneResetTimer = 0;
*/
    for(i in touches)
    {
        var x = Math.round(touches[i].pos().x());
        var y = Math.round(touches[i].pos().y());
        
        touchDeltaX = x - prevFrameMouseX;
        touchDeltaY = y - prevFrameMouseY;

        prevFrameMouseX = x;
        prevFrameMouseY = y;
        break; // for now, just use the pos from the first touch
    }
    
    if (numFingers == 2 && tilting == false)
    {
        //Log("Tilting true");
        moving = false;
        tilting = true;
    }
    else
        tilting = false;
}

function OnTouchEnd(e)
{
//    Log("UiCamera OnTouchEnd");
    lastTouchTimestamp = frame.WallClockTime();
    StopMovement();
    prevFrameMouseX = prevFrameMouseY = -1;
}

function ResetCamera()
{
    lastTouchTimestamp = frame.WallClockTime();
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
        SwitchScene(); // Resets the scene
}

// TODO: copy-paste from ObjectMove.js
function IsObjectMovable(e)
{
    return e.placeable && !e.terrain && e.dynamiccomponent && !(e.dynamiccomponent && e.dynamiccomponent.name == "Icon");
}

function HandleGestureEvent(e)
{
    Log("HandleGestureEvent :" + e.eventType);
}

function HandleMouseEvent(e)
{
    if (!me.camera.IsActive())
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
            var relY = useTouchInput ? touchDeltaY : e.relativeY;//e.y - prevFrameMouseY; // e.relativeY;
            //Log("tilting relY " + relY);
            var transform = me.placeable.transform;
            transform.rot.x -= cameraData.rotate.sensitivity * relY/*e.relativeY*/;
            var oldRotX = transform.rot.x;
            transform.rot.x = Clamp(transform.rot.x, minTiltAngle, maxTiltAngle);
            me.placeable.transform = transform;

            if (oldRotX > minTiltAngle && oldRotX < maxTiltAngle)
            {
                var d = relY/*e.relativeY*/ * cMoveZSpeed * 30;
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
            var d = Clamp(e.relativeZ/30, -5, 5);
            var newPos = me.placeable.WorldPosition();
            var dir = me.placeable.Orientation().Mul(scene.ForwardVector()).Normalized();
            var r = scene.ogre.Raycast(new Ray(newPos, dir), -1);
            newPos = newPos.Add(dir.Mul(d));
            me.placeable.SetPosition(newPos);
            // TODO Use minDistanceFromGround and maxDistanceFromGround 
        }
        break;
    case 3: // MousePressed
        if (e.button == 1)
        {
            var result = scene.ogre.Raycast(e.x, e.y);
            //if (result.entity && !result.entity.dynamiccomponent && result.entity.dynamiccomponent.name != "Icon" && !result.entity.graphicsviewcanvas)
            moving = !(result.entity && IsObjectMovable(result.entity));
            Log("ghgfhgfhgfhgfhgfhgfhgfh " + moving);
        }
        else if (e.button == 2)
        {
            var result = scene.ogre.Raycast(e.x, e.y);
            //if (result.entity && result.entity.dynamiccomponent && result.entity.dynamiccomponent.name != "Icon" && !result.entity.graphicsviewcanvas)
            tilting = !(result.entity && IsObjectMovable(result.entity));
            Log("fdshjdsfhsfd");
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

const /*float*/ ZoomSpeed = 0.02;
const /*float*/ TranslateSpeed = 0.012;

function DepthCameraUpdate()
{
/*
    if (!selectedObject)
    {
        if (Input.touchCount == 2)
        {
            if (lastDistance == 0)
            {
                Rect left = WallRemoval.LeftTop;
                Rect right = WallRemoval.RightTop;
                if (left.Contains(Input.touches[0].position) || right.Contains(Input.touches[0].position) ||
                    left.Contains(Input.touches[1].position) || right.Contains(Input.touches[1].position))
                {
                    lastDistance = -1;
                }
                else
                    lastDistance = Vector2.Distance(Input.touches[0].position, Input.touches[1].position);
            }
            else if (Input.touches[0].phase == TouchPhase.Moved && Input.touches[1].phase == TouchPhase.Moved && lastDistance > 0)
            {
                float distance = Vector2.Distance(Input.touches[0].position, Input.touches[1].position);
                transform.Translate(Vector3.left * (lastDistance - distance) *
                    (DepthSelect.ReferenceHeight / Screen.height) * ZoomSpeed, Space.World);
                lastDistance = distance;
            }
        }
        else
            lastDistance = 0;

        if (Input.touchCount == 3 && Input.touches[0].phase == TouchPhase.Moved && 
            Input.touches[1].phase == TouchPhase.Moved && Input.touches[2].phase == TouchPhase.Moved)
        {
            Vector2 avgDelta = (Input.touches[0].deltaPosition + Input.touches[1].deltaPosition + Input.touches[2].deltaPosition) / 3;
            transform.Translate(new Vector3(0, -avgDelta.y * (DepthSelect.ReferenceHeight / Screen.height) * TranslateSpeed,
                avgDelta.x * (DepthSelect.ReferenceHeight / Screen.height) * TranslateSpeed), Space.World);

            transform.LookAt(new Vector3(transform.position.x + 15, 2, transform.position.z * 0.2f));
        }
    }
*/
}
