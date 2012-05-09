// UiCamera.js - 3D UI Camera

// !ref: ScreenPrefab.txml
// !ref: Scene1.txml
// !ref: Scene2.txml
// !ref: Scene3.txml
// !ref: Scene4.txml

var sceneIndex = -1;
// Scene rotation
const scenes = ["Scene1.txml", "Scene2.txml", "Scene3.txml" ,"Scene4.txml" ];
var currentContent = [];
var calendarWidget = null;

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

if (!framework.IsHeadless())
{
    frame.DelayedExecute(1).Triggered.connect(ApplyCamera);
    me.Action("ResetCamera").Triggered.connect(ResetCamera);
}

function CreateUserWidgets()
{
/*
    var ents = scene.LoadSceneXML(asset.GetAsset("ScreenPrefab.txml").DiskSource(), false, false, 0);
    if (ents.length == 0)
    {
        LogE("Could not instantiate ScreenPrefab.txml");
        return;
    }

    ents[0].name = "Calendar";
    var t = ents[0].placeable.transform;
    t.pos = new float3(6.0, 1.25, -10.0);
    t.rot = new float3(90.0, -90.0, 90.0);
    ents[0].placeable.transform = t;

    var p = ents[0].placeable.parentRef;
    p.ref = me.id;
    ents[0].placeable.parentRef = p;

    var calendar = scene.EntityByName("calendar");
    calendarWidget = new QCalendarWidget();
    calendarWidget.gridVisible = true;
    calendar.graphicsviewcanvas.GraphicsScene().addWidget(calendarWidget);
    calendarWidget.show();
*/
}

function ApplyCamera()
{
//    me.camera.nearPlane = 5.0; // Try to push the near plane as far as possible so that objects faraway have as little Z fighting as possible.
//    scene.EntityByName("FreeLookCamera").camera.nearPlane = 5.0;
//    scene.EntityByName("FreeLookCamera").soundlistener.active = false;
    me.camera.SetActive();
    me.soundlistener.active = true;
    ResetCamera();

    frame.Updated.connect(Update);

//    frame.DelayedExecute(5).Triggered.connect(CreateUserWidgets);

    // Load the first scene.
    ++sceneIndex;
    SwitchScene();

    input.TouchBegin.connect(OnTouchBegin);
    input.TouchUpdate.connect(OnTouchUpdate);
    input.TouchEnd.connect(OnTouchEnd);
}

// TODO touch input
function OnTouchBegin(e) { }
function OnTouchUpdate(e) { }
function OnTouchEnd(e) { }

function ResetCamera()
{
//    lastTouchTimestamp = frame.WallClockTime();
    var resetTr = scene.EntityByName("UiCameraCameraSpawnPos").placeable.transform;
//    var resetTr = scene.EntityByName("FreeLookCameraSpawnPos").placeable.transform;
    me.placeable.transform = resetTr;
}

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

//const cMoveZSpeed = 0.005; // was 0.0007 in Unity
//const cRotateSpeed = 1;
//const cReferenceHeight = 768;

var moving = false; // Is camera in moving state
var tilting = false; // Is camera in tilting state

function OnScriptDestroyed()
{
    input.UnregisterInputContextRaw("UiCamera");
    if (calendarWidget && !framework.IsExiting())
        calendarWidget.deleteLater();
}

if (!server.IsRunning())
{
    engine.ImportExtension("qt.core");
    engine.ImportExtension("qt.gui");
    engine.IncludeFile("Log.js");
    engine.IncludeFile("MathUtils.js");

    ic = input.RegisterInputContextRaw("UiCamera", 102);
    ic.KeyEventReceived.connect(HandleKeyEvent);
    ic.MouseEventReceived.connect(HandleMouseEvent);
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
}

const minTiltAngle = 1.0;
const maxTiltAngle = 70.0;
const tiltSpeed = 0.22;
// The plane on which the camera tilts
var tiltPlane;
// Point on the globe the camera is looking at
var cameraLookAtPosition;// = new float3(0,0,0);
var globePosition = null; 

function UpdateRotateParams()
{
    return; // TODO
    cameraPos = renderer.MainCamera().placeable.WorldPosition();
    var cameraFwd = renderer.MainCamera().placeable.transform.Orientation().Mul(scene.ForwardVector());
    var ray = new Ray(cameraPos, cameraFwd);
    cameraLookAtPosition = ray.GetPoint(10);

    var mousePos = input.MousePos();
    var relX = mousePos.x()/ui.GraphicsScene().width();
    var relY = mousePos.y()/ui.GraphicsScene().height();
    var mouseRay = renderer.MainCameraComponent().GetMouseRay(relX, relY);
    var pos = mouseRay.GetPoint(10);

    tiltPlane = new Plane(cameraLookAtPosition, cameraPos, pos);

    return;
    var result = scene.ogre.Raycast(ray, -1);
    if (result.entity)
    {
        var entityPos = result.entity.placeable ? result.entity.placeable.WorldPosition() : result.pos;
        cameraLookAtPosition = result.pos;
        tiltPlane = new Plane(cameraLookAtPosition, cameraPos, entityPos);
    }
    else
    {
        cameraLookAtPosition = ray.GetPoint(200);
        tiltPlane = new Plane(cameraLookAtPosition, cameraPos, cameraLookAtPosition);
    }
}
// TODO: copy-paste from ObjectMove.js
function IsObjectMovable(e)
{
    return e.placeable && !e.terrain && !(e.dynamiccomponent && e.dynamiccomponent.name == "Icon");
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
            HandleMove(e.relativeX, e.relativeY, 0);
        }
        else if (tilting)
        {
            return;
//            Log("globePosition " + globePosition);
//            Log("cameraLookAtPosition " + cameraLookAtPosition);

            var cameraPos = renderer.MainCamera().placeable.WorldPosition();

            var v1 = cameraLookAtPosition.Sub(globePosition);
            var v2 =  cameraPos.Sub(cameraLookAtPosition);
            // BUG: AngleBetween buggy, returns NaN for unnormalized vectors
            //var cameraAngle = v1.AngleBetween(v2);
            var cameraAngle = RadToDeg(v1.Normalized().AngleBetweenNorm(v2.Normalized()));

            Log(e.relativeY);
            cameraAngle -= e.relativeY * tiltSpeed;
            cameraAngle = Clamp(cameraAngle, minTiltAngle, maxTiltAngle)
//            Log("cameraAngle should be" + cameraAngle);

            var targetUpVector = cameraLookAtPosition.Sub(globePosition);
            targetUpVector.Normalize();
//            Log("AAAAAAAAA " + targetUpVector);
            var l = cameraPos.Sub(cameraLookAtPosition).Length();
//            Log("l " + l);
            targetUpVector = targetUpVector.Mul(l);
//            Log("BBBBBBBBBBB " + targetUpVector);
            targetUpVector = new Quat(tiltPlane.normal, DegToRad(cameraAngle)).Mul(targetUpVector);
//            Log("CCCCCCCCCCCC " + targetUpVector);
            cameraPos = cameraLookAtPosition.Add(targetUpVector);
//            Log("cameraPos should be " + cameraPos);
            var t = renderer.MainCamera().placeable.transform;
            //t.pos = cameraPos;
            //Log("t.pos was " + t.pos);
            
            //Log("t.pos IS " + t.pos);
//            Quat.LookAt(localForward, targetDirection, localUp, worldUp);
            var targetLookatDir = cameraLookAtPosition.Sub(cameraPos).Normalized();
            t.rot = Quat.LookAt(scene.ForwardVector(), targetLookatDir, scene.UpVector(), scene.UpVector());
            renderer.MainCamera().placeable.transform = t;

            //endRotation = Quat(float3x4.LookAt(new float3(0,0,-1), targetLookatDirection, new float3(0,1,0), new float3(0,1,0)));

        /* OLD TEMP CODE
            var transform = me.placeable.transform;
            transform.rot.x -= cameraData.rotate.sensitivity * e.relativeY;
            transform.rot.x = Clamp(transform.rot.x, -90.0, 90.0);
            me.placeable.transform = transform;
        */
        }
        break;
    case 2: // MouseScroll
        if (moving)
            HandleMove(0, 0, e.relativeZ);
        break;
    case 3: // MousePressed
        if (e.button == 1)
        {
            var result = scene.ogre.Raycast(e.x, e.y);
            //if (result.entity && !result.entity.dynamiccomponent && result.entity.dynamiccomponent.name != "Icon" && !result.entity.graphicsviewcanvas)
            moving = !(result.entity && IsObjectMovable(result.entity));
        }
        else if (e.button == 2)
        {
            var result = scene.ogre.Raycast(e.x, e.y);
            //if (result.entity && result.entity.dynamiccomponent && result.entity.dynamiccomponent.name != "Icon" && !result.entity.graphicsviewcanvas)
            tilting = !(result.entity && IsObjectMovable(result.entity));
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
        moving = false;
        tilting = false;
        StopMovement();
        UpdateRotateParams();
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
        UpdateRotateParams();
    }
}

var timer = 0;

function Update(frametime)
{
    if (!me.camera.IsActive())
        return;

/*
    scene.ogre.DebugDrawSoundSource(cameraLookAtPosition, 1, 10, 1,0,0);
    if (globePosition);
        scene.ogre.DebugDrawSoundSource(globePosition, 1, 10, 0,0,1);
    if (tiltPlane)
        scene.ogre.DebugDrawPlane(tiltPlane, 0,0,1);
    if (targetUpVector)
*/
    if (cameraData.move.amount.x == 0 && cameraData.move.amount.y == 0 && cameraData.move.amount.z == 0)
        return;
/* DEBUG
    cameraPos = renderer.MainCamera().placeable.WorldPosition();
    var cameraFwd = renderer.MainCamera().placeable.transform.Orientation().Mul(scene.ForwardVector());
    var ray = new Ray(cameraPos, cameraFwd);
    var r = scene.ogre.Raycast(ray, -1);
    if (r.entity)
        Log("hit " + r.entity.name);
    else
        Log("no hit");
    scene.ogre.DebugDrawLine(ray.pos, ray.dir.Mul(300), 1,0,0);
*/

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
    cameraData.move.amount.z = /*-1 * */ deltaY;
    // right/left
    cameraData.move.amount.x = /*-1 * */ deltaX;
    // up/down
    var scroll = deltaZ/60;
    cameraData.move.amount.y = Clamp(scroll, -5, 5);
}

function StopMovement()
{
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

