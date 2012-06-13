// IconScript.js - handles animation of 3D icons and info bubbles

// !ref: InfoBubblePrefab.txml

engine.IncludeFile("Log.js");
engine.IncludeFile("MathUtils.js");

function MovingIcon(icon, start, dest)
{
    this.icon = icon;
    this.start = start;
    this.dest = dest;
    this.t = 0;
}

function InfoBubbleVisibilityInterpolation(icon)
{
    this.icon = icon;
    this.t = 0;
}

var infoBubbles = [];
var autoShownInfoBubbles = [];
var movingIcons = [];
var cam = null;
const cDefaultScale = 1;
const cVisibleScale = 8;
const cHiddenScale = 0.01; 
const cScaleTime = 1;
const cToggleInfoBubbleVisibilityTreshold = 250 * 250; // Squared distance treshold

// Entry point
if (!framework.IsHeadless())
{
    sceneinteract.EntityClicked.connect(OnEntityClicked);
    frame.Updated.connect(Update)
}

function OnEntityClicked(entity)
{
    if (IsEntityAnIcon(entity))
        ToggleInfoBubbleVisibility(entity);
}

function IsEntityAnIcon(e)
{
    return e.placeable != null && e.mesh != null && e.dynamiccomponent != null && e.dynamiccomponent.name == "Icon";
}

function ToggleInfoBubbleVisibility(icon)
{
    infoBubbles.push(new InfoBubbleVisibilityInterpolation(icon));

    var infoBubbleVisible = icon.dynamiccomponent.GetAttribute("infoBubbleVisible");
    var infoBubbleId = icon.dynamiccomponent.GetAttribute("infoBubbleId");
    if (infoBubbleVisible != undefined && infoBubbleVisible) // Info bubble visible, remove it.
    {
        var infoBubble = scene.EntityById(infoBubbleId);
        if (!infoBubble)
        {
            LogE("Trying to remove non-existing info bubble with entity ID " + infoBubbleId);
            return;
        }
        // TODO: crashes to EC_GraphicsViewCanvas::OnMaterialChanged. Fow now, only hiding the entity.
        //scene.RemoveEntity(infoBubbleId);
        //infoBubble.placeable.visible = false;
        icon.dynamiccomponent.SetAttribute("infoBubbleVisible", !infoBubbleVisible);

        // Moving of icons disabled for now
//        var currentPos = icon.placeable.WorldPosition();
//        movingIcons.push(new MovingIcon(icon, currentPos, currentPos.Add(new float3(0,-70,0))));
    }
    else
    {
        if (infoBubbleId == undefined) // Info bubble not created, create it now.
        {
            var ents = scene.LoadSceneXML(asset.GetAsset("InfoBubblePrefab.txml").DiskSource(), false, false, 0);
            if (ents.length == 0)
            {
                LogE("Could not instantiate InfoBubblePrefab.txml");
                return;
            }

            var infoBubble = ents[0];
            infoBubble.temporary = true;
            infoBubble.placeable.SetPosition(icon.placeable.WorldPosition());

            icon.dynamiccomponent.CreateAttribute("bool", "infoBubbleVisible");
            icon.dynamiccomponent.SetAttribute("infoBubbleVisible", !infoBubbleVisible);
            icon.dynamiccomponent.CreateAttribute("uint", "infoBubbleId");
            icon.dynamiccomponent.SetAttribute("infoBubbleId", infoBubble.id);

            infoBubbleId = infoBubble.id;
        }

        var infoBubble = scene.EntityById(infoBubbleId);
        if (!infoBubble)
        {
            LogE("Trying to set visibility of a non-existing info bubble with entity ID "+ infoBubbleId);
            return;
        }

        //infoBubble.placeable.visible = true;
        icon.dynamiccomponent.SetAttribute("infoBubbleVisible", true/*infoBubble.placeable.visible*/);

        // Moving of icons disabled for now
//        var currentPos = icon.placeable.WorldPosition();
//        movingIcons.push(new MovingIcon(icon, currentPos, currentPos.Add(new float3(0,70,0))));
    }
}

function DesiredObjectScale(mesh)
{
    if (!cam)
        return 1;
    if (!cam.placeable)
        return 1;

    var objectPos = mesh.LocalToWorld().TranslatePart();
    var cameraPos = cam.placeable.LocalToWorld().TranslatePart();
    var distance = objectPos.Distance(cameraPos);
    const multiplier = 0.05;
    return Math.max(multiplier, multiplier*distance);
}

function AnimateInfoBubbleScale(dt)
{
    for(i = 0; i < infoBubbles.length; ++i)
    {
        var e = infoBubbles[i].icon;
        var infoBubbleVisible = e.dynamiccomponent.GetAttribute("infoBubbleVisible");
        var infoBubbleId = e.dynamiccomponent.GetAttribute("infoBubbleId");
        var infoBubble = scene.EntityById(infoBubbleId);
        var scale = infoBubble.mesh.nodeTransformation.scale.x;
        if (infoBubbleVisible)
        {
            if (!EqualAbs(scale, cVisibleScale, 0.001))
                infoBubbles[i].t += dt;
            else
            {
                infoBubbles.splice(i, 1);
                infoBubble.placeable.visible = infoBubbleVisible;
            }

            if (!EqualAbs(scale, cVisibleScale, 0.001) && infoBubbles[i].t < cScaleTime)
            {
                scale = Lerp(cHiddenScale, cVisibleScale, infoBubbles[i].t/cScaleTime);
                infoBubble.mesh.SetAdjustScale(float3.FromScalar(scale));
            }
        }
        else
        {
            if (!EqualAbs(scale, cHiddenScale, 0.001))
                infoBubbles[i].t += dt;
            else
            {
                infoBubbles.splice(i, 1);
                infoBubble.placeable.visible = infoBubbleVisible;
            }

            if (!EqualAbs(scale, cHiddenScale, 0.001) && infoBubbles[i].t < cScaleTime)
            {
                scale = Lerp(cVisibleScale, cHiddenScale, infoBubbles[i].t/cScaleTime);
                infoBubble.mesh.SetAdjustScale(float3.FromScalar(scale));
            }
        }
    }

    // Interpolate moving icons
    // Disabled for now
/*
    for(i = 0; i < movingIcons.length; ++i)
    {
        var currentPos = movingIcons[i].icon.placeable.WorldPosition();
        var distance = currentPos.Distance(movingIcons[i].dest);
        if (distance > 1)
        {
            movingIcons[i].t += dt;
        }
        else
        {
            movingIcons[i].icon.placeable.SetPosition(movingIcons[i].dest);
            movingIcons.splice(i, 1);
        }

        if (movingIcons[i] && distance > 0.1 && movingIcons[i].t < cScaleTime)
        {
            currentPos = float3.Lerp(movingIcons[i].start, movingIcons[i].dest, movingIcons[i].t/cScaleTime);
            movingIcons[i].icon.placeable.SetPosition(currentPos);
        }
    }
*/
}

function Update(dt)
{
    if (!cam)
        cam = renderer.MainCamera();
    if (!cam)
        return;

    var entities = scene.Entities();
    for(i in entities)
    {
        var e = entities[i];
        if (IsEntityAnIcon(e))
        {
            var entityId = parseInt(e.id); // WTF NOTE: typeof(e.id) is object, not number, so must convert it explicitly here
            var idx = autoShownInfoBubbles.indexOf(entityId);
            if (e.placeable.WorldPosition().DistanceSq(cam.placeable.WorldPosition()) < cToggleInfoBubbleVisibilityTreshold)
            {
                if (idx == -1)
                {
                    autoShownInfoBubbles.push(entityId);
                    ToggleInfoBubbleVisibility(e);
                }
            }
            else
            {
                if (idx != -1)
                {
                    ToggleInfoBubbleVisibility(e);
                    autoShownInfoBubbles.splice(idx, 1);
                }
            }

            // Uncomment to enable auto-rotation and -scaling
/*
            var dir = e.placeable.WorldPosition().Sub(cam.placeable.WorldPosition()).Normalized();
            var q = Quat.LookAt(scene.ForwardVector(), dir, scene.UpVector(), scene.UpVector());
            e.placeable.SetOrientation(q);

            // Auto-scale
            e.mesh.SetAdjustScale(float3.FromScalar(DesiredObjectScale(e.mesh)));
*/
        }
    }

    AnimateInfoBubbleScale(dt);
}
