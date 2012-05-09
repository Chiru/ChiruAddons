// LookAtCamera.js - Script for facing object to camera.

// Note: the following values are for the news mesh, not applicable for all objects.
const localForward = new float3(-1, 0, 0);
const localUp = new float3(0, -1, 0);
const worldUp = scene.UpVector();

frame.Updated.connect(LookAtCamera)

function LookAtCamera()
{
    var cam = renderer.MainCamera();
    if (cam)
    {
        var dir = me.placeable.WorldPosition().Sub(cam.placeable.WorldPosition()).Normalized();
        var q = Quat.LookAt(localForward, dir, localUp, worldUp);
        me.placeable.SetOrientation(q);
    }
}
