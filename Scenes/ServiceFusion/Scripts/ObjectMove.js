// !ref: InfoBubblePrefab.txml

/*
 * Object
  - Object selection - long press
  - moving object
   * object rotation (yaw, pitch) - select object, second finger y-axis drag
   * object roll - select object, rotate second finger around object
   * "Zoom" object (depth movement) - select object, one finger bottom-right corner of the screen, second finger y-axis drag
*/

const cMoveZSpeed = 0.005; // was 0.0007 in Unity
const cRotateSpeed = 1;
const cReferenceHeight = 768;
const longTouchTime = 2;
var selectedObject = null;

var touchOffset = new float3(0,0,0);
var touchInputActive = false;
var prevFrameMouseX = -1;
var prevFrameMouseY = -1;
var touchDeltaX, touchDeltaY;

var prevFrameTouches;

function OnScriptDestroyed()
{
    input.UnregisterInputContextRaw("3dUiObjectMove");
}

// Entry point
if (!framework.IsHeadless())
{
    engine.ImportExtension("qt.core");
    engine.ImportExtension("qt.gui");
    engine.IncludeFile("Log.js");
    engine.IncludeFile("MathUtils.js");
    engine.IncludeFile("Utils.js");

    var ic = input.RegisterInputContextRaw("3dUiObjectMove", 90);
    ic.KeyEventReceived.connect(HandleKeyEvent);

    input.TouchBegin.connect(OnTouchBegin);
    input.TouchUpdate.connect(OnTouchUpdate);
    input.TouchEnd.connect(OnTouchEnd);

    ui.GraphicsView().DragEnterEvent.connect(HandleDragEnterEvent);
    ui.GraphicsView().DragMoveEvent.connect(HandleDragMoveEvent);
    ui.GraphicsView().DropEvent.connect(HandleDropEvent);
    
    frame.Updated.connect(Update);
}

function BeginMove()
{
    var ray = CurrentMouseRay();
//    scene.ogre.DebugDrawLine(ray.pos, ray.dir.Mul(200), 1,0,0);
    var r = scene.ogre.Raycast(ray, -1);
    if (r.entity && IsObjectMovable(r.entity))
        selectedObject = r.entity;
    else
        selectedObject = null;

    if (selectedObject) // calculate click/touch offset
    {
        var cameraEntity = renderer.MainCamera();
//        var cameraEntity = scene.EntityByName("UiCamera");

        var camFwd = cameraEntity.placeable.WorldOrientation().Mul(scene.ForwardVector());
        var orientedPlane = new Plane(camFwd.Mul(-1), 0);
        var movePlane = new Plane(camFwd, orientedPlane.Distance(selectedObject.placeable.WorldPosition()));
        var r = IntersectRayPlane(movePlane, ray);
        if (r.intersects)
        {
            var moveTo = ray.GetPoint(r.distance);
            touchOffset = selectedObject.placeable.WorldPosition().Sub(moveTo);
        }

        // TODO Highlighting of selected object
    }
}

function EndMove()
{
    selectedObject = null;
}

function OnTouchUpdate(e)
{
    prevFrameTouches = e.touchPoints();
//    Log("ObjectMove OnTouchUpdate " + e.touchPoints().length);
    touchInputActive = true;
/*
    var touches = e.touchPoints();
    var touchCount = touches.length;
    for(i in touches)
    {
        if (touches[i].state() == Qt.TouchPointStationary)
            LogE("jajajajajajjaja");
        
        var x = Math.round(touches[i].pos().x());
        var y = Math.round(touches[i].pos().y());
        
        touchDeltaX = x - prevFrameMouseX;
        touchDeltaY = y - prevFrameMouseY;

//        deltaX = deltaX * Math.min(3.0, Math.max(1.0, Math.abs(deltaX) / 3.0));
//        deltaRotation += deltaX / ui.GraphicsView().width;
//        cameraPitchAmount = Math.max(cameraPitchMin, Math.min(cameraPitchMax, cameraPitchAmount + deltaY / ui.GraphicsView().height));

        prevFrameMouseX = x;
        prevFrameMouseY = y;
        //break;
    }
*/
}

function OnTouchBegin(e)
{
    prevFrameTouches = e.touchPoints();
    touchInputActive = true;
    // Must enforce calling of TouchUpdate here, otherwise we get no events with 'pressed' state.
    TouchUpdate();
/*
    var touches = e.touchPoints();
    var numFingers = touches.length;
    for(i in touches)
    {
        prevFrameMouseX = touches[i].pos().x();
        prevFrameMouseY = touches[i].pos().y();
        break;
    }
*/
//    TouchUpdate();
}

function OnTouchEnd(e)
{
//    Log("ObjectMove OnTouchEnd");
    prevFrameTouches = e.touchPoints();
    frame.DelayedExecute(0.1).Triggered.connect(function(){prevFrameTouches=[];});
    touchInputActive = false;
    //TouchUpdate();
}

function HandleDragEnterEvent(e)
{
    var data = e.mimeData().data("application/x-hotspot").toString();
    if (data.length > 0)
        e.acceptProposedAction();
}

function HandleDragMoveEvent(e)
{
    var data = e.mimeData().data("application/x-hotspot").toString();
    if (data.length > 0)
        e.acceptProposedAction();
}

function CurrentMouseRay()
{
    var x, y;
    if (touchInputActive)
    {
        x = prevFrameMouseX, y = prevFrameMouseY;
    }
    else
    {
        var mousePos = input.MousePos();
        x = mousePos.x(), y = mousePos.y();
    }
    return renderer.MainCameraComponent().GetMouseRay(x/ui.GraphicsScene().width(), y/ui.GraphicsScene().height());
}

function ScreenPointToRay(x, y)
{
    return renderer.MainCameraComponent().GetMouseRay(x/ui.GraphicsScene().width(), y/ui.GraphicsScene().height());
}

function HandleDropEvent(e)
{
    var data = e.mimeData().data("application/x-hotspot").toString();
    if (data.length > 0)
    {
        var raycast = scene.ogre.Raycast(CurrentMouseRay(), -1);
        if (raycast.entity && raycast.entity.graphicsviewcanvas)
            return; // Raycast hit screen, use it instead creating a new one.

        var camEntity = renderer.MainCamera();
        var preferredDistance = 20.0;
        var rayStart = camEntity.placeable.WorldPosition();
        var rayDir = camEntity.placeable.WorldOrientation().Mul(scene.ForwardVector()).Normalized();

        var ray = CurrentMouseRay(); //new Ray(rayStart, rayDir);
        var res = scene.ogre.Raycast(ray, -1);
        if (res.entity)
        {
            var hitDistance = res.pos.Sub(rayStart).Length() - 1.0;
            if (hitDistance < 2.5)
            {
                Log("No room for new screen!");
                return;
            }
            if (hitDistance < preferredDistance)
                preferredDistance = hitDistance;
        }

        var pos = ray.GetPoint(preferredDistance)
//        var worldOffsetVec = camEntity.placeable.transform.Orientation().Mul(new float3(0, 1.5, -preferredDistance));

        var ents = scene.LoadSceneXML(asset.GetAsset("InfoBubblePrefab.txml").DiskSource(), false, false, 0);
        if (ents.length == 0)
        {
            LogE("Could not instantiate InfoBubblePrefab.txml");
            return;
        }

//        Log("HandleDropEvent: Creating new screen at " + pos)
        ents[0].placeable.SetPosition(pos);
        var uiWidget = new DragDropWidget(null);
        uiWidget.size = new QSize(ents[0].graphicsviewcanvas.width, ents[0].graphicsviewcanvas.height);
        uiWidget.setLayout(new QVBoxLayout());
        var newLabel = new QLabel(e.mimeData().text(), uiWidget);
        // TODO Apply style sheet
//    var styleSheet = ByteArrayToString(e.mimeData().data("application/stylesheet"));
//        if (styleSheet.length > 0)
//            newLabel.styleSheet = styleSheet;
        uiWidget.layout().addWidget(newLabel, 0, 0);
        ents[0].graphicsviewcanvas.GraphicsScene().addWidget(uiWidget);
        uiWidget.show();
    }
}

function HandleKeyEvent(e)
{
    if (e.HasCtrlModifier() && e.keyCode == Qt.Key_E && selectedObject)
    {
        var t = selectedObject.placeable.transform;
        t.rot = new float3(180.0, 90.0, 0.0);
        selectedObject.placeable.transform = t;
    }
}

var mousePosPrev = null;//input.MousePos();
var rotAngle = 0;

// Handles touch input
function TouchUpdate()
{
    if (!prevFrameTouches/* || prevFrameTouches.length == 0*/)
    {
        //Log("no touches " + frame.WallClockTime());
        return;
    }
//    Log("TouchUpdate " + prevFrameTouches.length + " " + frame.WallClockTime());
//    touchInputActive = true;
    var touches = prevFrameTouches;
    var touchCount = touches.length;

    for(i in touches)
    {
        var x = Math.round(touches[i].pos().x());
        var y = Math.round(touches[i].pos().y());
        touchDeltaX = x - prevFrameMouseX;
        touchDeltaY = y - prevFrameMouseY;
        prevFrameMouseX = x;
        prevFrameMouseY = y;
        break;
    }

    TouchSelectObject(touchCount, touches, null);
    //TouchMoveObject(touchCount, touches, null);
    //TouchRotateObject(touchCount, touches, null);
}

// Handles mouse + keyboard
function Update(/*frameTime*/)
{
    TouchUpdate();
    //return;
//    if (input.IsMouseButtonPressed(1))
//        BeginMove();

    if (touchInputActive || input.IsMouseButtonDown(1))
    {
        if (selectedObject)
        {
            var move = touchInputActive ? true : input.IsKeyDown(Qt.Key_1);
            var rotate = input.IsKeyDown(Qt.Key_2);
            if (move)
            {
                if (input.IsKeyDown(Qt.Key_Control)) // Z movement
                {
                    if (selectedObject.dynamiccomponent && selectedObject.dynamiccomponent.name == "UserItem")
                        return; // Zooming/depth movement of user items not allowed

                    var mouseYDelta = mousePosPrev.y() - input.MousePos().y();
                    var d = mouseYDelta * cMoveZSpeed * 30;

                    var cameraEntity = renderer.MainCamera();

                    var newPos = selectedObject.placeable.Position();
                    var direction = selectedObject.placeable.WorldPosition().Sub(cameraEntity.placeable.WorldPosition()).Normalized();
                    // TODO mesh.OBB()
                    //var nearestPoint = selectedObject.placeable.collider.ClosestPointOnBounds(cameraEntity.placeable.Position());
                    //nearestPoint.Add(direction.Mul(d));

                    // TODO: parented objects
                    //if (selectedObject.ObjectTransform.parent)
                        //direction = selectedObject.ObjectTransform.parent.InverseTransformDirection(direction);

                    newPos = newPos.Add(direction.Mul(d));

                    if (!CanObjectBeMoved(selectedObject, newPos))
                        return;

                    //var globalNewPos = selectedObject.ObjectTransform.TransformPoint(newPos);

                    //var distanceToCamera = float3.Distance(nearestPoint, cameraEntity.Position());
                    //if ((d > 0 && distanceToCamera < 0.9 * transform.localScale.x) || (d < 0 && distanceToCamera > 0.01 * transform.localScale.x))
                        selectedObject.placeable.SetPosition(newPos);
                }
                else // X & Y movement
                    MoveSelected(input.MousePos());
            }
            if (rotate)
            {
                if (input.IsMouseButtonDown(1))
                {
                    if (selectedObject.dynamiccomponent && selectedObject.dynamiccomponent.name == "UserItem")
                        return; // Zooming/depth movement of user items not allowed
                    var d = new float2(mousePosPrev.x()-input.MousePos().x(), mousePosPrev.y()-input.MousePos().y());
                    d = d.Mul(cReferenceHeight / ui.GraphicsScene().height() * cRotateSpeed /* *50 */); // *50 in Unity commented out

                    var rotVectorFor = selectedObject.placeable.Position().Sub(renderer.MainCamera().placeable.Position()).Normalized();
                    rotVectorFor = rotVectorFor.Cross(scene.UpVector());
                    var rotVectorRig = scene.UpVector();

                    var q = selectedObject.placeable.transform.Orientation();
                    var a = Quat.RotateAxisAngle(rotVectorRig.Normalized(), DegToRad(-1 * d.x));
                    var b = Quat.RotateAxisAngle(rotVectorFor.Normalized(), DegToRad(-1 * d.y));
                    q = q.Mul(a).Mul(b);
                    selectedObject.placeable.SetOrientation(q.Normalized());
                }
            }
        }
    }

    mousePosPrev = input.MousePos();
}

function CanObjectBeMoved(obj, pos)
{
    var obb = obj.mesh.WorldOBB();
    obb.pos = pos;

//    scene.ogre.DebugDrawOBB(obb, 0,1,0);
    var hits = scene.physics.ObbCollisionQuery(obb)
    if (hits.length == 0)
        return true;

    for(i = 0; i < hits.length; ++i)
        if (hits[i] != obj)
        {
//            scene.ogre.DebugDrawOBB(hits[i].mesh.WorldOBB(), 1,0,0);
            return false;
        }

    return true;
}

// TODO/NOTE pos param unused
function MoveSelected(/*QPoint*/pos)
{
    if (selectedObject)
    {
        var ray = CurrentMouseRay();
        var cameraEntity = renderer.MainCamera();
//        var cameraEntity = scene.EntityByName("UiCamera");
        var camFwd = cameraEntity.placeable.WorldOrientation().Mul(scene.ForwardVector()).Normalized();
        var orientedPlane = new Plane(camFwd, 0);
        //camFwd = camFwd.Mul(-1);
        var movePlane = new Plane(camFwd, orientedPlane.Distance(selectedObject.placeable.WorldPosition()));

//        scene.ogre.DebugDrawLine(ray.pos, ray.dir.Mul(200), 1,0,0);
//        scene.ogre.DebugDrawPlane(orientedPlane, 0,0,1);
//        scene.ogre.DebugDrawPlane(movePlane, 0,1,0);

        var r = IntersectRayPlane(movePlane, ray);
        if (r.intersects)
        {
            var moveTo = ray.GetPoint(r.distance);

            moveTo = moveTo.Add(touchOffset);

            var parent = selectedObject.placeable.ParentPlaceableComponent();
            if (parent)
                moveTo = parent.WorldToLocal().MulPos(moveTo);

            if (!CanObjectBeMoved(selectedObject, moveTo.Add(selectedObject.mesh.nodeTransformation.pos)))
                return;

            selectedObject.placeable.SetPosition(moveTo);
        }
    }
}

// TODO: set these variables in StartMove
// Alternatively/preferably have selectedObject Object which stores all of the following information
var selectedObjectRotationOrigin;
var selectedObjectFingerId;
var selectedObjectOriginalRotation;
var selectedObjectObjectTransform;

const cLongTouchDuration = 2.0;

var startPositions = {}; //associative array <int (finger ID), QPointF (screen pos)>
var rotAngle = 0; // float

function TouchRotateObject(touchCount, touches, e)
{
    return; // TODO Implement TouchRotate
    for(i in touches)
        if (touches[i].state() == Qt.TouchPointPressed)
            startPositions[t.id()] = touches[i].pos();

    if (selectedObject && touchCount == 2)
    {
        var stationaryTouch = null; // QTouchEvent::TouchPoint
        var moveTouch = null; // QTouchEvent::TouchPoint
        if (touches[0].state() == Qt.TouchPointStationary)
            stationaryTouch = touches[0];
        if (touches[0].state() == Qt.TouchPointMoved)
            moveTouch = touches[0];
        if (touches[1].state() == Qt.TouchPointStationary)
            stationaryTouch = touches[1];
        if (touches[1].state() == Qt.TouchPointMoved)
            moveTouch = touches[1];

        if (!stationaryTouch)
        {
            if (DistanceQPointF(touches[0].pos(), startPositions[touches[0].id()]) < 20)
                stationaryTouch = touches[0];
            else if (DistanceQPointF(touches[1].pos(), startPositions[touches[1].id()]) < 20)
                stationaryTouch = touches[1];
        }

        var /*float*/ touchesDistance = DistanceQPointF(touches[0].pos(), touches[1].pos());
        if (stationaryTouch && moveTouch && stationaryTouch.id() == selectedObjectFingerId &&
            !ViewportAreas.LeftBottom.Contains(moveTouch.Value.position) &&
            touchesDistance < UISelect.MoveZMinTouchDistance * (UISelect.ReferenceHeight / Screen.height))
        {
            if (selectedObjectRotationOrigin == Vector2.zero)
            {
                selectedObjectRotationOrigin = SubQPointF(moveTouch.pos(), stationaryTouch.pos());
                selectedObjectOriginalRotation = selectedObjectObjectTransform.rotation;
            }
            else
            {
                var /*float2*/ rotDir = SubQPointF(moveTouch.pos(), stationaryTouch.pos());
                var /*float*/ angle = float2.Angle(selectedObjectRotationOrigin, rotDir);
                var /*float3*/ cross = float3.Cross(selectedObjectRotationOrigin, rotDir);
                if (cross.z < 0)
                    angle = 360 - angle;

                selectedObjectObjectTransform.rotation = selectedObjectOriginalRotation;
                var /*float3*/ forward = selectedObject.placeable.WorldPosition().Sub(renderer.MainCamera().placeable.WorldPosition()).Normalized();
                selectedObjectObjectTransform.Rotate(forward, angle, Space.World);

                if (touches[0].id() == selectedObjectFingerId)
                    print(touchOffset);
                    // TODO selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[0].pos(), selectedObjectObjectTransform, selectedObject.OnPlane);
                else if (touches[1].id() == selectedObjectFingerId)
                    // TODO selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[1].pos(), selectedObjectObjectTransform, selectedObject.OnPlane);
                    print(touchOffset);
            }
        }
        else if (touches[0].state() == Qt.TouchPointMoved && touches[1].state() == Qt.TouchPointMoved)
        {
            var delta0 = SubQPointF(touches[0].pos(), touches[0].lastPos());
            var delta1 = SubQPointF(touches[1].pos(), touches[1].lastPos());
            var /*float2*/ d = delta0.Add(delta1).Div(2.0).Mul(cReferenceHeight/ui.GraphicsScene().height()).Mul(cRotateSpeed);

            var /*float3*/ forward = (selectedObjectObjectTransform.position - Camera.main.transform.position).normalized;
            var /*float3*/right = Vector3.Cross(forward, Camera.main.transform.up);
            var /*float3*/ up = Camera.main.transform.up;

            selectedObjectObjectTransform.Rotate(up, -d.x, Space.World);
            selectedObjectObjectTransform.Rotate(right, -d.y, Space.World);

            if (touches[0].id() == selectedObjectFingerId)
                selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[0].pos(), selectedObjectObjectTransform, selectedObject.OnPlane);
            else if (touches[1].id() == selectedObjectFingerId)
                selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[1].pos(), selectedObjectObjectTransform, selectedObject.OnPlane);

            selectedObjectRotationOrigin = new float2(0, 0);
        }
    }
}

function TouchMoveObject(touchCount, touches, e)
{
    for(i in touches)
        if (touches[i].state() == Qt.TouchPointPressed)
            startPositions[touches[i].id()] = touches[i].pos();

    // TODO: is the following needed?
//    if (touchCount > 1 && uiSelect.IsSelected)
//        singleTouch = false;
//    else if (touchCount == 0 && !singleTouch)
//        singleTouch = true;

    if (touchCount == 1)// && singleTouch)
        if (selectedObject && touches[0].state() == Qt.TouchPointMoved) // TODO && !ViewportAreas.LeftBottom.Contains(touches[0].pos()))
            MoveSelected(touches[0].pos());

    return;

    if (selectedObject && touchCount == 2)
    {
        var stationaryTouch = null; // QTouchEvent::TouchPoint
        var moveTouch = null; // QTouchEvent::TouchPoint
        if (touches[0].state() == Qt.TouchPointStationary)
            stationaryTouch = touches[0];
        if (touches[0].state() == Qt.TouchPointMoved)
            moveTouch = touches[0];
        if (touches[1].state() == Qt.TouchPointStationary)
            stationaryTouch = touches[1];
        if (touches[1].state() == Qt.TouchPointMoved)
            moveTouch = touches[1];

        if (!stationaryTouch)
        {
            if (DistanceQPointF(touches[0].pos(), startPositions[touches[0].id()]) < 20)
                stationaryTouch = touches[0];
            else if (DistanceQPointF(touches[1].pos(), startPositions[touches[1].id()]) < 20)
                stationaryTouch = touches[1];
        }

        var touchesDistance = DistanceQPointF(touches[0].pos(), touches[1].pos());
//        if (touchesDistance > UISelect.MoveZMinTouchDistance * (UISelect.ReferenceHeight / Screen.height))
        /*
        {
        // TODO
            if (stationaryTouch && moveTouch && ViewportAreas.LeftBottom.Contains(stationaryTouch.Value.position))
            {
                var d = moveTouch.Value.deltaPosition.y * (UISelect.ReferenceHeight / Screen.height) * moveZSpeed * transform.localScale.x;

                var newPos = selectedObjectObjectTransform.localPosition;
                var direction = (selectedObjectObjectTransform.position - Camera.main.transform.position).normalized;
                if (selectedObjectObjectTransform.parent)
                    direction = selectedObjectObjectTransform.parent.InverseTransformDirection(direction);

                newPos += direction * d;

                Plane farplane = new Plane(-direction, Camera.main.transform.position + direction * 0.1f * transform.localScale.x);
                Plane nearplane = new Plane(direction, Camera.main.transform.position + direction * 0.012f);

                var nearestPoint = selectedObjectObjectTransform.collider.ClosestPointOnBounds(Camera.main.transform.position);
                var  distanceToCamera = Vector3.Distance(nearestPoint, Camera.main.transform.position);

                if ((d > 0 && distanceToCamera < 0.9f * transform.localScale.x) || (d < 0 && distanceToCamera > 0.2122f))
                    selectedObjectObjectTransform.localPosition = newPos;

                if (DistanceQPointF(moveTouch.pos(), startPositions[moveTouch.Value.id()]) > 20)
                    selectedObject.TouchOffset = UISelect.CalculateMoveOffset(moveTouch.pos(), selectedObjectObjectTransform, selectedObject.OnPlane);
            }
        }
        */
    }
}

var longTouchStartTime = 0;
var longTouchStartTimePos = null;

//var singleTouch = true;
function TouchSelectObject(touchCount, touches, e)
{
//    if (touchCount > 1 && selectedObject == null)
//        singleTouch = false;
//    else if (touchCount == 0 && !singleTouch)
//        singleTouch = true;

    if (selectedObject)
        for(i in touches)
            if (touches[i].state() == Qt.TouchPointReleased ||  touches[i].state() == Qt.TouchPointPressed)
            {
                selectedObjectRotationOrigin = new float2(0, 0);
                selectedObjectOriginalRotation = new Quat(0, 0, 0, 1);
                break;
            }

    for(i in touches)
    {
        if (touches[i].state() == Qt.TouchPointPressed)
            startPositions[touches[i].id()] = touches[i].pos();
    }
    if (touchCount == 1 /*&& singleTouch*/)
    {
        if (touches[0].state() == Qt.TouchPointPressed)
        {
            longTouchStartTime = frame.WallClockTime();
            longTouchStartTimePos = touches[0].pos();
        }
        else if (IsTouchStateMoved(touches[0]))
        {
            longTouchStartTime = 0;
        }
        else if (longTouchStartTimePos && IsTouchStateStationary(touches[0]) &&
            frame.WallClockTime() - longTouchStartTime > cLongTouchDuration && !selectedObject)
        {
            longTouchStartTime = 0;
            if (DistanceQPointF(longTouchStartTimePos, touches[0].pos()) < 30)
            {
                BeginMove();
                //TODO
                //StartMove(0,0,0);
                //Transform highlighted = FindNearestObject(touches[0].pos());
                //if (highlighted)
                //    StartMove(highlighted, touches[0].pos(), touches[0].id());
            }
        }
    }
    
    if (touchCount == 2 && !selectedObject)
    {
        var t = touches[0];
        var t1 = false;//ViewportAreas.LeftBottom.Contains(touches[0].pos());
        var t2 = false;//ViewportAreas.LeftBottom.Contains(touches[1].pos());
        if (t1)
            t = touches[1];
        
        if (t1 || t2)
        {
            //Transform highlighted = FindNearestObject(t.pos());
            //if (highlighted)
            //    StartMove(highlighted, t.pos(), t.id());
            StartMove();
        }
    }
    
    if (touchCount == 0 && selectedObject)
        EndMove();
}

