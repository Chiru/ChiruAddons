// AutoScale.js - Script for keeping object constant-sized in the view.

frame.Updated.connect(AutoUpdateScale)

function AutoUpdateScale()
{
    me.mesh.SetAdjustScale(float3.FromScalar(DesiredObjectScale()));
}

function DesiredObjectScale()
{
    var cam = renderer.MainCamera();
    if (!cam)
        return 1;
    if (!cam.placeable)
        return 1;

    var objectPos = me.mesh.LocalToWorld().TranslatePart();
    var cameraPos = cam.placeable.LocalToWorld().TranslatePart();
    var distance = objectPos.Distance(cameraPos);
    return Math.max(0.1, 0.1*distance);
}
