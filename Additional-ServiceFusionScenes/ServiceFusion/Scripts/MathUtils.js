// MathUtils.js - Math utility functions copy-paste-adapted from MathGeoLib

function Clamp(val, floor, ceil) { return val <= ceil ? (val >= floor ? val : floor) : ceil; }

function Clamp01(val) { return Clamp(val, 0, 1); }

function Lerp(a, b, t) { return a + t * (b-a); }

function DegToRad(degrees) { return degrees * (Math.PI / 180); }

function RadToDeg(radians) { return radians * (180 / Math.PI); }

function Dot(a, b) { return a.Dot(b); }

function Abs(a) { return a >= 0 ? a : -a; }

function EqualAbs(a, b, epsilon) { return Abs(a-b) < epsilon; }

function IntersectLinePlane(ptOnPlane, planeNormal, lineStart, lineDir)
{
    var result = {};

    var denom = Dot(lineDir, planeNormal);
    if (EqualAbs(denom, 0))
    {
        result.intersects = false; // Either we have no intersection, or the whole line is on the plane. @todo distinguish these cases.
        result.distance = -1; // put negative value to denote failure
    }
    else
    {
        result.intersects = true;
        result.distance = Dot(ptOnPlane.Sub(lineStart), planeNormal) / denom;
    }
    return result;
}

function IntersectRayPlane(plane, ray)
{
    return IntersectLinePlane(plane.PointOnPlane(), plane.normal, ray.pos, ray.dir);
}

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

// Returns whether the values of two QPoint(F)s are equal.
function EqualsQPointF(pos1, pos2)
{
    return pos1.x() == pos2.x() && pos1.y() == pos2.y();
}
