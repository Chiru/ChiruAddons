// TouchInput.js - Coverting the touch input code from Unity W.I.P.

private Dictionary<int, Vector2> startPositions = new Dictionary<int, Vector2>();
private float rotAngle = 0;

void TouchRotate(/*QTouchEvent*/ e)
{
    var touches = e.touchPoints();
    var touchCount = touches.length;
    for(i in touches)
        if (touches[i].state() == Qt.TouchPointPressed)
            startPositions[t.id()] = .pos();

    if (selectedObject && touchCount == 2)
    {
        var /*QTouchEvent::TouchPoint*/ stationaryTouch = null;
        var /*QTouchEvent::TouchPoint*/ moveTouch = null;
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
            if (Vector3.Distance(touches[0].pos(), startPositions[touches[0].id()]) < 20)
                stationaryTouch = touches[0];
            else if (Vector3.Distance(touches[1].pos(), startPositions[touches[1].id()]) < 20)
                stationaryTouch = touches[1];
        }

        float touchesDistance = Vector2.Distance(touches[0].pos(), touches[1].pos());
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
                Vector2 rotDir = moveTouch.Value.position - stationaryTouch.Value.position;
                float angle = Vector2.Angle(selectedObject.RotationOrigin, rotDir);
                Vector3 cross = Vector3.Cross(selectedObject.RotationOrigin, rotDir);
                if (cross.z < 0)
                    angle = 360 - angle;

                selectedObject.ObjectTransform.rotation = selectedObject.OriginalRotation;
                Vector3 forward = (selectedObject.ObjectTransform.position - Camera.main.transform.position).normalized;
                selectedObject.ObjectTransform.Rotate(forward, angle, Space.World);

                if (touches[0].id() == selectedObject.FingerId)
                    selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[0].pos(), selectedObject.ObjectTransform, selectedObject.OnPlane);
                else if (touches[1].id() == selectedObject.FingerId)
                    selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[1].pos(), selectedObject.ObjectTransform, selectedObject.OnPlane);
            }
        }
        else if (touches[0].state() == Qt.TouchPointMoved && touches[1].state() == Qt.TouchPointMoved)
        {
            Vector2 d = ((touches[0].deltaPosition + touches[1].deltaPosition) / 2f) * (UISelect.ReferenceHeight / Screen.height) * UISelect.RotateSpeed;

            Vector3 forward = (selectedObject.ObjectTransform.position - Camera.main.transform.position).normalized;
            Vector3 right = Vector3.Cross(forward, Camera.main.transform.up);
            Vector3 up = Camera.main.transform.up;

            selectedObject.ObjectTransform.Rotate(up, -d.x, Space.World);
            selectedObject.ObjectTransform.Rotate(right, -d.y, Space.World);

            if (touches[0].id() == selectedObject.FingerId)
                selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[0].pos(), selectedObject.ObjectTransform, selectedObject.OnPlane);
            else if (touches[1].id() == selectedObject.FingerId)
                selectedObject.TouchOffset = UISelect.CalculateMoveOffset(touches[1].pos(), selectedObject.ObjectTransform, selectedObject.OnPlane);

            selectedObject.RotationOrigin = Vector2.zero;
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
        var /*QTouchEvent::TouchPoin*/ stationaryTouch = null;
        var /*QTouchEvent::TouchPoin*/ moveTouch = null;
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
            if (Vector3.Distance(touches[0].pos(), startPositions[touches[0].id()]) < 20)
                stationaryTouch = touches[0];
            else if (Vector3.Distance(touches[1].pos(), startPositions[touches[1].id()]) < 20)
                stationaryTouch = touches[1];
        }

        float touchesDistance = Vector2.Distance(touches[0].pos(), touches[1].pos());
    //	if (touchesDistance > UISelect.MoveZMinTouchDistance * (UISelect.ReferenceHeight / Screen.height))
        {
            if (stationaryTouch.HasValue && moveTouch.HasValue && ViewportAreas.LeftBottom.Contains(stationaryTouch.Value.position))
            {
                float d = moveTouch.Value.deltaPosition.y * (UISelect.ReferenceHeight / Screen.height) * moveZSpeed * transform.localScale.x;

                Vector3 newPos = selectedObject.ObjectTransform.localPosition;
                Vector3 direction = (selectedObject.ObjectTransform.position - Camera.main.transform.position).normalized;
                if (selectedObject.ObjectTransform.parent)
                    direction = selectedObject.ObjectTransform.parent.InverseTransformDirection(direction);

                newPos += direction * d;

                Plane farplane = new Plane(-direction, Camera.main.transform.position + direction * 0.1f * transform.localScale.x);
                Plane nearplane = new Plane(direction, Camera.main.transform.position + direction * 0.012f);

                Vector3 nearestPoint = selectedObject.ObjectTransform.collider.ClosestPointOnBounds(Camera.main.transform.position);
                float distanceToCamera = Vector3.Distance(nearestPoint, Camera.main.transform.position);

                if ((d > 0 && distanceToCamera < 0.9f * transform.localScale.x) || (d < 0 && distanceToCamera > 0.2122f))
                    selectedObject.ObjectTransform.localPosition = newPos;

                if (Vector3.Distance(moveTouch.Value.position, startPositions[moveTouch.Value.id()]) > 20)
                    selectedObject.TouchOffset = UISelect.CalculateMoveOffset(moveTouch.Value.position, selectedObject.ObjectTransform, selectedObject.OnPlane);
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
                selectedObject.RotationOrigin = Vector2.zero;
                selectedObject.OriginalRotation = Quaternion.identity;
                break;
            }

    foreach(Touch t in touches)
        if (t.state() == Qt.TouchPointPressed)
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
