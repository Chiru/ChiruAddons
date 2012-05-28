// TouchInput.js - Coverting the touch input code from Unity W.I.P.

function DistanceQPointF(/*QPoint*/ pos1, /*QPoint*/ pos2)
{
    return new float2(pos1.x(), pos1.y()).Distance(float2(pos2.x(), pos2.y())
}

function SubQPointF(/*QPoint*/ pos1, /*QPoint*/ pos2)
{
    return new float2(pos1.x() - pos2.x(), pos1.y() - pos2.y());
}

//private Dictionary<int, Vector2>
var startPositions = {}; //new Dictionary<int, Vector2>();
var rotAngle = 0; // float

void TouchRotate(/*QTouchEvent*/ e)
{
    var touches = e.touchPoints();
    var touchCount = touches.length;
    for(i in touches)
        if (touches[i].state() == Qt.TouchPointPressed)
            startPositions[t.id()] = .pos();

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

        float touchesDistance = DistanceQPointF(touches[0].pos(), touches[1].pos());
        if (stationaryTouch.HasValue && moveTouch.HasValue &&
            stationaryTouch.Value.id() == selectedObject.FingerId &&
            !ViewportAreas.LeftBottom.Contains(moveTouch.Value.position) &&
            touchesDistance < UISelect.MoveZMinTouchDistance * (UISelect.ReferenceHeight / Screen.height))
        {
            if (selectedObject.RotationOrigin == Vector2.zero)
            {
                selectedObject.RotationOrigin = moveTouch.Value.position - stationaryTouch.Value.position;
                selectedObject.OriginalRotation = selectedObject.ObjectTransform.rotation;
            }
            else
            {
                var /*float2*/ rotDir = SubQPointF(moveTouch.pos(), stationaryTouch.pos());
                float angle = float2.Angle(selectedObject.RotationOrigin, rotDir);
                var /*float3*/ cross = float3.Cross(selectedObject.RotationOrigin, rotDir);
                if (cross.z < 0)
                    angle = 360 - angle;

                selectedObject.ObjectTransform.rotation = selectedObject.OriginalRotation;
                var /*float3*/ forward = selectedObject.ObjectTransform.position.Sub(Camera.main.transform.position).Normalized();
                selectedObject.ObjectTransform.Rotate(forward, angle, Space.World);

                if (touches[0].id() == selectedObject.FingerId)
                    selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[0].pos(), selectedObject.ObjectTransform, selectedObject.OnPlane);
                else if (touches[1].id() == selectedObject.FingerId)
                    selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[1].pos(), selectedObject.ObjectTransform, selectedObject.OnPlane);
            }
        }
        else if (touches[0].state() == Qt.TouchPointMoved && touches[1].state() == Qt.TouchPointMoved)
        {
            var /*float2*/ d = ((touches[0].deltaPosition + touches[1].deltaPosition) / 2f) * (UISelect.ReferenceHeight / Screen.height) * UISelect.RotateSpeed;

            var /*float3*/ forward = (selectedObject.ObjectTransform.position - Camera.main.transform.position).normalized;
            var /*float3*/right = Vector3.Cross(forward, Camera.main.transform.up);
            var /*float3*/ up = Camera.main.transform.up;

            selectedObject.ObjectTransform.Rotate(up, -d.x, Space.World);
            selectedObject.ObjectTransform.Rotate(right, -d.y, Space.World);

            if (touches[0].id() == selectedObject.FingerId)
                selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[0].pos(), selectedObject.ObjectTransform, selectedObject.OnPlane);
            else if (touches[1].id() == selectedObject.FingerId)
                selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[1].pos(), selectedObject.ObjectTransform, selectedObject.OnPlane);

            selectedObject.RotationOrigin = new float2(0, 0);
        }
    }
}

function TouchMove(/*QTouchEvent*/ e)
{
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
    {
        if (uiSelect.IsSelected && touches[0].state() == Qt.TouchPointMoved && !ViewportAreas.LeftBottom.Contains(touches[0].pos()))
            MoveSelected(touches[0].pos());
    }

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

        float touchesDistance = DistanceQPointF(touches[0].pos(), touches[1].pos());
    //	if (touchesDistance > UISelect.MoveZMinTouchDistance * (UISelect.ReferenceHeight / Screen.height))
        {
            if (stationaryTouch.HasValue && moveTouch.HasValue && ViewportAreas.LeftBottom.Contains(stationaryTouch.Value.position))
            {
                var /*float*/ d = moveTouch.Value.deltaPosition.y * (UISelect.ReferenceHeight / Screen.height) * moveZSpeed * transform.localScale.x;

                var /*float3*/ newPos = selectedObject.ObjectTransform.localPosition;
                var /*float3*/ direction = (selectedObject.ObjectTransform.position - Camera.main.transform.position).normalized;
                if (selectedObject.ObjectTransform.parent)
                    direction = selectedObject.ObjectTransform.parent.InverseTransformDirection(direction);

                newPos += direction * d;

                Plane farplane = new Plane(-direction, Camera.main.transform.position + direction * 0.1f * transform.localScale.x);
                Plane nearplane = new Plane(direction, Camera.main.transform.position + direction * 0.012f);

                var /*float3*/ nearestPoint = selectedObject.ObjectTransform.collider.ClosestPointOnBounds(Camera.main.transform.position);
                var /*float*/ distanceToCamera = Vector3.Distance(nearestPoint, Camera.main.transform.position);

                if ((d > 0 && distanceToCamera < 0.9f * transform.localScale.x) || (d < 0 && distanceToCamera > 0.2122f))
                    selectedObject.ObjectTransform.localPosition = newPos;

                if (DistanceQPointF(moveTouch.pos(), startPositions[moveTouch.Value.id()]) > 20)
                    selectedObject.TouchOffset = UISelect.CalculateMoveOffset(moveTouch.pos(), selectedObject.ObjectTransform, selectedObject.OnPlane);
            }
        }
    }
}

function Select(/*QTouchEvent*/ e)
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
                selectedObject.RotationOrigin = new float2(0, 0);
                selectedObject.OriginalRotation = new Quat(0, 0, 0, 1);
                break;
            }

    foreach(i in touches)
        if (touches[i].state() == Qt.TouchPointPressed)
            startPositions[t.id()] = .pos();

    if (touchCount == 1 && singleTouch)
    {
        if (touches[0].state() == Qt.TouchPointPressed)
        {
            longtouchStart = Time.time;
            longtouchStartPos = touches[0].pos();
        }
        else if (touches[0].state() == Qt.TouchPointMoved)
        {
            longtouchStart = 0;
        }
        else if (touches[0].state() == Qt.TouchPointStationary && Time.time - longtouchStart > LongTouchDuration && !IsSelected)
        {
            longtouchStart = 0;

            if (Vector2.Distance(longtouchStartPos, touches[0].pos()) < 30)
            {
                Transform highlighted = FindNearestObject(touches[0].pos());
                if (highlighted)
                    StartMove(highlighted, touches[0].pos(), touches[0].id());
            }
        }
    }
    
    if (touchCount == 2 && !IsSelected)
    {
        Touch t = touches[0];
        bool t1 = ViewportAreas.LeftBottom.Contains(touches[0].pos());
        bool t2 = ViewportAreas.LeftBottom.Contains(touches[1].pos());
        if (t1)
            t = touches[1];
        
        if (t1 || t2)
        {
            Transform highlighted = FindNearestObject(.pos());
            if (highlighted)
                StartMove(highlighted, .pos(), t.id());
        }
    }
    
    if (touchCount == 0 && IsSelected)
        StopMove();
}
