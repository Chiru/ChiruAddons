// TouchInput.js - Coverting the touch input code from Unity W.I.P.

// TODO: set these variables in StartMove
// Alternatively/preferably have selectedObject Object which stores all of the following information
var selectedObjectRotationOrigin;
var selectedObjectFingerId;
var selectedObjectOriginalRotation;
var selectedObjectObjectTransform;

const cLongTouchDuration = 1.0;

// Returns distance between two QPoint(F)s.
function DistanceQPointF(/*QPoint(F)*/ pos1, /*QPoint(F)*/ pos2)
{
    return new float2(pos1.x(), pos1.y()).Distance(new float2(pos2.x(), pos2.y()));
}

// Substracts two QPoint(F)s and returns the result as float2.
function SubQPointF(/*QPoint(F)*/ pos1, /*QPoint(F)*/ pos2)
{
    return new float2(pos1.x() - pos2.x(), pos1.y() - pos2.y());
}

var startPositions = {}; //new Dictionary<int, Vector2>();
var rotAngle = 0; // float

function TouchRotate(/*QTouchEvent*/ e)
{
    return; // TODO Implement TouchRotate
    var touches = e.touchPoints();
    var touchCount = touches.length;
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

function TouchMove(/*QTouchEvent*/ e)
{
    return; // TODO Implement TouchMove
    
    var touches = e.touchPoints();
    var touchCount = touches.length;
    foreach(i in touches)
        if (touches[i].state() == Qt.TouchPointPressed)
            startPositions[t.id()] = touches[i].pos();

    if (touchCount > 1 && uiSelect.IsSelected)
        singleTouch = false;
    else if (touchCount == 0 && !singleTouch)
        singleTouch = true;

    if (touchCount == 1)// && singleTouch)
        if (selectedObject && touches[0].state() == Qt.TouchPointMoved) // TODO && !ViewportAreas.LeftBottom.Contains(touches[0].pos()))
            MoveSelected(touches[0].pos());

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
    }*/
}

function TouchSelect(/*QTouchEvent*/ e)
{
    var touches = e.touchPoints();
    var touchCount = touches.length;
    if (touchCount > 1 && selectedObject == null)
        singleTouch = false;
    else if (touchCount == 0 && !singleTouch)
        singleTouch = true;

    if (selectedObject)
        for(i in touches)
            if (touches[i].state() == Qt.TouchPointReleased || /*touches[i].state() == TouchPhase.Canceled ||*/ touches[i].state() == Qt.TouchPointPressed)
            {
                selectedObjectRotationOrigin = new float2(0, 0);
                selectedObjectOriginalRotation = new Quat(0, 0, 0, 1);
                break;
            }

    foreach(i in touches)
        if (touches[i].state() == Qt.TouchPointPressed)
            startPositions[touches[i].id()] = touches[i].pos();

    if (touchCount == 1 && singleTouch)
    {
        if (touches[0].state() == Qt.TouchPointPressed)
        {
            longtouchStart = frame.WallClockTime();
            longtouchStartPos = touches[0].pos();
        }
        else if (touches[0].state() == Qt.TouchPointMoved)
        {
            longtouchStart = 0;
        }
        else if (touches[0].state() == Qt.TouchPointStationary && frame.WallClockTime() - longtouchStart > cLongTouchDuration && !selectedObject)
        {
            longtouchStart = 0;

            if (DistanceQPointF(longtouchStartPos, touches[0].pos()) < 30)
            {
                //TODO
                StartMove(0,0,0);
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
        StopMove();
}

function StartMove(highlighted, position, id)
{
    //console.LogInfo("StartMove");
}

function StopMove()
{
    //console.LogInfo("StopMove");
}
}
