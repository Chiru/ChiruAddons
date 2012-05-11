// !ref: ScreenPrefab.txml

const cMoveZSpeed = 0.005; // was 0.0007 in Unity
const cRotateSpeed = 1;
const cReferenceHeight = 768;

var selectedObject = null;

var touchOffset = new float3(0,0,0);

var lastTouchTimestamp = frame.WallClockTime();

function OnScriptDestroyed()
{
    input.UnregisterInputContextRaw("3dUiObjectMove");
}

// Entry point
if (!server.IsRunning())
{
    engine.ImportExtension("qt.core");
    engine.ImportExtension("qt.gui");
    engine.IncludeFile("Log.js");
    engine.IncludeFile("MathUtils.js");
//    engine.IncludeFile("copy.js");
    frame.Updated.connect(Update);

    var ic = input.RegisterInputContextRaw("3dUiObjectMove", 90);
    ic.KeyEventReceived.connect(HandleKeyEvent);

    input.TouchBegin.connect(OnTouchBegin);
    input.TouchUpdate.connect(OnTouchUpdate);
    input.TouchEnd.connect(OnTouchEnd);

    ui.GraphicsView().DragEnterEvent.connect(HandleDragEnterEvent);
    ui.GraphicsView().DragMoveEvent.connect(HandleDragMoveEvent);
    ui.GraphicsView().DropEvent.connect(HandleDropEvent);
}

// TODO touch input
function OnTouchBegin(e)
{
    Log("OnTouchBegin " + e.touchPoints().length);
    lastTouchTimestamp = frame.WallClockTime();
/*
    var touches = e.touchPoints();
    for(i in touches)
    {
        prevFrameMouseX = touches[i].pos().x();
        prevFrameMouseY = touches[i].pos().y();
        return;
    }
*/
}

function OnTouchUpdate(e)
{
    Log("OnTouchUpdate " + e.touchPoints().length);
    lastTouchTimestamp = frame.WallClockTime();
/*
    var touches = e.touchPoints();
    for(i in touches)
    {
        var x = touches[i].pos().x();
        var y = touches[i].pos().y();

//    var uiitem = input.ItemAtCoords(x,y);
 //     if (uiitem == null)
  //    {
        var deltaX = x - prevFrameMouseX;
        var deltaY = y - prevFrameMouseY;

        deltaX = deltaX * Math.min(3.0, Math.max(1.0, Math.abs(deltaX) / 3.0));

        deltaRotation += deltaX / ui.GraphicsView().width;

        cameraPitchAmount = Math.max(cameraPitchMin, Math.min(cameraPitchMax, cameraPitchAmount + deltaY / ui.GraphicsView().height));
//      }
 //   }
//    else
//       deltaRotation = 0;

        prevFrameMouseX = x;//input.MousePos().x();
        prevFrameMouseY = y;//input.MousePos().y();
        return;
    }
*/
}

function OnTouchEnd(e)
{
    Log("OnTouchEnd");
}

// QByteArray's toString not exposed in QtScript, must do it manually.
QByteArray.prototype.toString = function()
{
    return new QTextStream(this, QIODevice.ReadOnly).readAll();
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
    var mousePos = input.MousePos();
    var relX = mousePos.x()/ui.GraphicsScene().width();
    var relY = mousePos.y()/ui.GraphicsScene().height();
    return renderer.MainCameraComponent().GetMouseRay(relX, relY);
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

        var ents = scene.LoadSceneXML(asset.GetAsset("ScreenPrefab.txml").DiskSource(), false, false, 0);
        if (ents.length == 0)
        {
            LogE("Could not instantiate ScreenPrefab.txml");
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
        t.rot = new float3(-90.0, 90.0, 90.0)
        selectedObject.placeable.transform = t;
    }
}

var mousePosPrev = null;//input.MousePos();
var rotAngle = 0;
var selectionTimer = 0;

function IsObjectMovable(e)
{
    return e.placeable && !e.terrain && e.dynamiccomponent && !(e.dynamiccomponent && e.dynamiccomponent.name == "Icon");
}

function Update(frameTime)
{
    if (input.IsMouseButtonPressed(1))
    {
        var r = scene.ogre.Raycast(CurrentMouseRay(), -1);
        if (r.entity && IsObjectMovable(r.entity))
            selectedObject = r.entity;
        else
            selectedObject = null;

        if (selectedObject) // calculate click/touch offset
        {
            var cameraEntity = renderer.MainCamera();
    //        var cameraEntity = scene.EntityByName("UiCamera");
            var ray = CurrentMouseRay();
            var camFwd = cameraEntity.placeable.WorldOrientation().Mul(scene.ForwardVector());
            var orientedPlane = new Plane(camFwd.Mul(-1), 0);
            var movePlane = new Plane(camFwd, orientedPlane.Distance(selectedObject.placeable.WorldPosition()));
            var r = IntersectRayPlane(movePlane, ray);
            if (r.intersects)
            {
                var moveTo = ray.GetPoint(r.distance);
                touchOffset = selectedObject.placeable.WorldPosition().Sub(moveTo);
            }
        }
    }

    if (input.IsMouseButtonDown(1))
    {
//        Log("MouseButtonDown " + frameTime);
/*
        selectionTimer += frameTime;
        if (selectionTimer > 2)
        {
            var mousePos = input.MousePos();
            var result = scene.ogre.Raycast(mousePos.x(), mousePos.y());
            if (result.entity && result.entity.placeable)
                selectedObject = result.entity; //SelectObject(result.entity, true);
            else
                selectedObject = null;
                

            selectionTimer = 0;
        }
*/
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
    else
    {
        selectionTimer = 0;
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
