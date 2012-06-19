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
// Screen resolution of the target device will be 1200x800
const cReferenceHeight = 800;

var selectedObject = null;
var selectedObjectFingerId;
// TODO: set these variables in BeginMove
// Alternatively/preferably have selectedObject Object which stores all of the following information
var selectedObjectRotationOrigin;
var selectedObjectOriginalRotation;
var selectedObjectObjectTransform;

var touchOffset = new float3(0,0,0);
var prevFrameMouseX = -1;
var prevFrameMouseY = -1;
var prevFrameTouches;

var uiCamera = null;

function OnScriptDestroyed()
{
    input.UnregisterInputContextRaw("ObjectMove");
}

// Entry point
if (!framework.IsHeadless())
{
    engine.ImportExtension("qt.core");
    engine.ImportExtension("qt.gui");
    engine.IncludeFile("Log.js");
    engine.IncludeFile("MathUtils.js");
    engine.IncludeFile("Utils.js");

    var ic = input.RegisterInputContextRaw("ObjectMove", 90);
    ic.KeyEventReceived.connect(HandleKeyEvent);

    input.TouchBegin.connect(OnTouchBegin);
    input.TouchUpdate.connect(OnTouchUpdate);
    input.TouchEnd.connect(OnTouchEnd);

    ui.GraphicsView().DragEnterEvent.connect(HandleDragEnterEvent);
    ui.GraphicsView().DragMoveEvent.connect(HandleDragMoveEvent);
    //ui.GraphicsView().DropEvent.connect(HandleDropEvent);
    
    frame.Updated.connect(Update);
}

// Returns mouse ray from the active camera at last frame screen point coordinates.
function CurrentMouseRay()
{
    return MouseRay(prevFrameMouseX, prevFrameMouseY);
}

function SetSelectedObject(e)
{
    if (e != selectedObject)
    {
        selectedObject = e;

        // Signal object selection to the UI camera
        if (!uiCamera)
            uiCamera = scene.EntityByName("UiCamera");
        uiCamera.Exec(1, "ObjectSelected", e != null ? e.id.toString() : "0");
    }
}

function BeginMove(fingerId)
{
    var ray = CurrentMouseRay();
//    scene.ogre.DebugDrawLine(ray.pos, ray.dir.Mul(200), 1,0,0);
    var r = scene.ogre.Raycast(ray, -1);
    SetSelectedObject(r.entity && IsObjectMovable(r.entity) ? r.entity : null);

    if (selectedObject) // calculate click/touch offset
    {
        selectedObjectFingerId = fingerId;
        selectedObjectObjectTransform = selectedObject.placeable.transform;
        
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
    SetSelectedObject(null);
}

function OnTouchUpdate(e)
{
    prevFrameTouches = e.touchPoints();
/*
    var touches = e.touchPoints();
    var touchCount = touches.length;
    for(i in touches)
    {
//        Log(touches[i].state()); 
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

var useTouchInput = false;

function OnTouchBegin(e)
{
    useTouchInput = true;

    prevFrameTouches = e.touchPoints();
    // Must enforce calling of TouchUpdate here, otherwise we get no events with 'pressed' state.
    TouchUpdate();
}

function OnTouchEnd(e)
{
    prevFrameTouches = e.touchPoints();
    // TODO IS the line below needed?
    frame.DelayedExecute(0.1).Triggered.connect(function(){prevFrameTouches=[];});
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

/*function HandleDropEvent(e)
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
}*/

function HandleKeyEvent(e)
{
    if (e.HasCtrlModifier() && e.keyCode == Qt.Key_E && selectedObject)
    {
        var t = selectedObject.placeable.transform;
        t.rot = new float3(180.0, 90.0, 0.0);
        selectedObject.placeable.transform = t;
    }
}

// Handles touch input
function TouchUpdate()
{
    if (!prevFrameTouches/* || prevFrameTouches.length == 0*/)
        return;
 
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
    TouchMoveObject(touchCount, touches, null);
//    TouchRotateObject(touchCount, touches, null);
}

function Update(/*frameTime*/)
{
    if (useTouchInput)
    {
        TouchUpdate();
    }
    else // Rest of the function is the keyboard + mouse input code 
    {
        if (input.IsMouseButtonPressed(1))
            BeginMove(-1);
        if (input.IsMouseButtonReleased(1))
            EndMove();
        
        if (input.IsMouseButtonDown(1))
        {
            if (selectedObject)
            {
                var move = input.IsKeyDown(Qt.Key_1);
                var rotate = input.IsKeyDown(Qt.Key_2);
                if (move)
                {
                    if (input.IsKeyDown(Qt.Key_Control)) // Z movement
                    {
                        if (selectedObject.dynamiccomponent && selectedObject.dynamiccomponent.name == "UserItem")
                            return; // Zooming/depth movement of user items not allowed

                        var mouseYDelta = prevFrameMouseY - input.MousePos().y();
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
                        var d = new float2(prevFrameMouseX-input.MousePos().x(), prevFrameMouseY-input.MousePos().y());
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

        prevFrameMouseX = input.MousePos().x();
        prevFrameMouseY = input.MousePos().y();
    }
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

function MoveSelected(/*QPoint(F)*/pos)
{
    if (selectedObject)
    {
        var ray = MouseRay(pos.x(), pos.y());
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

const cLongTouchDuration = 1.0;
const cMoveZMinTouchDistance = 350;

var startPositions = {}; //associative array <int (finger ID), QPointF (screen pos)>
var rotAngle = 0; // float

function TouchRotateObject(touchCount, touches, e)
{
    for(i in touches)
        if (touches[i].state() == Qt.TouchPointPressed)
            startPositions[touches[i].id()] = touches[i].pos();

    if (selectedObject && touchCount == 2)
    {
        var stationaryTouch = null;
        var moveTouch = null;
        if (IsTouchStateStationary(touches[0]))
            stationaryTouch = touches[0];
        if (touches[0].state() == Qt.TouchPointMoved)
            moveTouch = touches[0];
        if (IsTouchStateStationary(touches[1]))
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
        if (stationaryTouch && moveTouch && stationaryTouch.id() == selectedObjectFingerId &&
            !IsPosWithinBottomLeftCorner(moveTouch.pos()) &&
            touchesDistance < cMoveZMinTouchDistance * (cReferenceHeight / Screen.height))
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

            var forward = selectedObjectObjectTransform.pos.Sub(uiCamera.placeable.transform.pos).Normalized();
            var right = forward.Cross(uiCamera.camera.upVector);
            var up = uiCamera.camera.upVector;

            // TODO
            //selectedObjectObjectTransform.Rotate(up, -d.x, Space.World);
            //selectedObjectObjectTransform.Rotate(right, -d.y, Space.World);

            if (touches[0].id() == selectedObjectFingerId)
                Log("todo1");
                //selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[0].pos(), selectedObjectObjectTransform, selectedObject.OnPlane);
            else if (touches[1].id() == selectedObjectFingerId)
                //selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[1].pos(), selectedObjectObjectTransform, selectedObject.OnPlane);
                Log("todo2");

            selectedObjectRotationOrigin = new float2(0, 0);
        }
    }
}

function IsPosWithinBottomLeftCorner(pos)
{
    return pos.x() < 200 && pos.y() < cReferenceHeight - 200;
}

function TouchMoveObject(touchCount, touches, e)
{
    for(i in touches)
        if (touches[i].state() == Qt.TouchPointPressed)
            startPositions[touches[i].id()] = touches[i].pos();

    // TODO: is the following needed?
//    if (touchCount > 1 && selectedObject)
//        singleTouch = false;
//    else if (touchCount == 0 && !singleTouch)
//        singleTouch = true;

    if (touchCount == 1)// && singleTouch)
        if (selectedObject && touches[0].state() == Qt.TouchPointMoved && touches[0].id() == selectedObjectFingerId
            && !IsPosWithinBottomLeftCorner(touches[0].pos()))
        {
            MoveSelected(touches[0].pos());
        }

    if (selectedObject && touchCount == 2)
    {
        var stationaryTouch = null;
        var moveTouch = null;
        if (IsTouchStateStationary(touches[0]))
            stationaryTouch = touches[0];
        if (touches[0].state() == Qt.TouchPointMoved)
            moveTouch = touches[0];
        if (IsTouchStateStationary(touches[1]))
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

        //if (stationaryTouch)
          //  Log(stationaryTouch.id());
        //var touchesDistance = DistanceQPointF(touches[0].pos(), touches[1].pos());
        //if (touchesDistance > cMoveZMinTouchDistance * cReferenceHeight / ui.GraphicsScene().height())
        {
            if (stationaryTouch && moveTouch && IsPosWithinBottomLeftCorner(stationaryTouch.pos()))
            {
                var d = (moveTouch.pos().y() - moveTouch.lastPos().y()) * (cReferenceHeight / ui.GraphicsScene().height()) * cMoveZSpeed * selectedObject.placeable.transform.scale.x;
                d *= 30;
                Log("d " + d);
                var cameraEntity = renderer.MainCamera();
                var newPos = selectedObject.placeable.Position();
                //Log("old pos " + newPos);
                var direction = selectedObject.placeable.WorldPosition().Sub(cameraEntity.placeable.WorldPosition()).Normalized();
                // TODO parented object
//                var parent = selectedObject.placeable.ParentPlaceableComponent();
//                if (parent)
                    //direction = parent.WorldToLocal().MulDir(direction);
//                    direction = selectedObjectObjectTransform.parent.InverseTransformDirection(direction);

                newPos = newPos.Add(direction.Mul(d));
                //Log("new pos " + newPos);
                //if (!CanObjectBeMoved(selectedObject, newPos))
                  //  return;
/*
                var Plane farplane = new Plane(-direction, Camera.main.transform.position + direction * 0.1f * transform.localScale.x);
                var Plane nearplane = new Plane(direction, Camera.main.transform.position + direction * 0.012f);

                var nearestPoint = selectedObjectObjectTransform.collider.ClosestPointOnBounds(Camera.main.transform.position);
                var  distanceToCamera = Vector3.Distance(nearestPoint, Camera.main.transform.position);

                if ((d > 0 && distanceToCamera < 0.9f * transform.localScale.x) || (d < 0 && distanceToCamera > 0.2122f))
                    selectedObjectObjectTransform.localPosition = newPos;
*/
                selectedObject.placeable.SetPosition(newPos);
                    // TODO?
//                if (DistanceQPointF(moveTouch.pos(), startPositions[moveTouch.id()]) > 20)
//                    selectedObject.TouchOffset = UISelect.CalculateMoveOffset(moveTouch.pos(), selectedObjectObjectTransform, selectedObject.OnPlane);
            }
        }
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
                BeginMove(touches[0].id());
                //TODO
                //StartMove(0,0,0);
                //Transform highlighted = FindNearestObject(touches[0].pos());
                //if (highlighted)
                //    StartMove(highlighted, touches[0].pos(), touches[0].id());
            }
        }
    }
    
    if (false /* TODO*/ && touchCount == 2 && !selectedObject)
    {
        Log("asdf");
        var t = touches[0];
        var t1 = IsPosWithinBottomLeftCorner(touches[0].pos());
        var t2 = IsPosWithinBottomLeftCorner(touches[1].pos());
        if (t1)
            t = touches[1];
        
        if (t1 || t2)
        {
            //Transform highlighted = FindNearestObject(t.pos());
            //if (highlighted)
            //    StartMove(highlighted, t.pos(), t.id());
            BeginMove(t.id());
        }
    }
    
    if (touchCount == 0 && selectedObject)
        EndMove();
}
