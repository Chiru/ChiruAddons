// IconScript.js - handles animation of 3D icons and info bubbles

// !ref: InfoBubblePrefab.txml

engine.IncludeFile("Log.js");
engine.IncludeFile("MathUtils.js");

function OnScriptDestroyed()
{
    // TODO proper cleanup
}

me.Action("RegisterInfoBubble").Triggered.connect(function(iconEntityName, infoBubbleId)
{
    var icon = scene.EntityByName(iconEntityName);
    if (!icon)
    {
        LogE("RegisterInfoBubble: icon null.");
        return;
    }
    var dc = icon.GetComponent("EC_DynamicComponent", "Icon");
    if (!dc)
    {
        LogE("RegisterInfoBubble: dc null.");
        return;
    }

    var infoBubble = scene.EntityById(parseInt(infoBubbleId));
    infoBubble.placeable.SetPosition(icon.placeable.WorldPosition());
    dc.CreateAttribute("uint", "infoBubbleId");
    dc.CreateAttribute("bool", "infoBubbleVisible");
    dc.SetAttribute("infoBubbleId", parseInt(infoBubbleId));
    dc.SetAttribute("infoBubbleVisible", true);
    // Enforce hide for newly registered info bubbles.
    SetInfoBubbleVisibility(icon, false);
});

function AnimatedIcon(icon, start, dest)
{
    this.icon = icon;
    this.start = start;
    this.dest = dest;
    this.t = 0;
}

function AnimatedInfoBubble(icon)
{
    this.icon = icon;
    this.t = 0;
}

var animatedInfoBubbles = [];
var autoShownInfoBubbles = [];
var animatedIcons = [];
var cam = null;
const cDefaultScale = 1;
const cVisibleScale = 16;
const cIconMoveFactor = 8.75;
const cIconMove = cVisibleScale * cIconMoveFactor;
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

// Returns whether or not the visibility was actually changed.
function SetInfoBubbleVisibility(icon, visible)
{
    var infoBubbleId = icon.dynamiccomponent.GetAttribute("infoBubbleId");
    var infoBubbleVisible = icon.dynamiccomponent.GetAttribute("infoBubbleVisible");
    //if (visible == infoBubbleVisible)
      //  return false;

    var currentPos = icon.placeable.WorldPosition(), destPos;

    if (visible)
    {
        if (infoBubbleId == undefined) // Info bubble not created, create it now.
        {
            var ents = scene.LoadSceneXML(asset.GetAsset("InfoBubblePrefab.txml").DiskSource(), false, false, 0);
            if (ents.length == 0)
            {
                LogE("Could not instantiate InfoBubblePrefab.txml");
                return false;
            }

            var infoBubble = ents[0];
            infoBubble.temporary = true;
            infoBubble.placeable.SetPosition(icon.placeable.WorldPosition());

            icon.dynamiccomponent.CreateAttribute("bool", "infoBubbleVisible");
            icon.dynamiccomponent.CreateAttribute("uint", "infoBubbleId");
            icon.dynamiccomponent.SetAttribute("infoBubbleId", infoBubble.id);

            infoBubbleId = infoBubble.id;
        }

        var infoBubble = scene.EntityById(infoBubbleId);
        if (!infoBubble)
        {
            LogE("Trying to set visibility of a non-existing info bubble with entity ID "+ infoBubbleId);
            return false;
        }

        icon.dynamiccomponent.SetAttribute("infoBubbleVisible", true);

        destPos = currentPos.Add(new float3(0, cIconMove, 0));
    }
    else
    {
        var infoBubble = scene.EntityById(infoBubbleId);
        if (!infoBubble)
        {
            LogE("Trying to remove non-existing info bubble with entity ID " + infoBubbleId);
            return false;
        }
        // TODO: crashes to EC_GraphicsViewCanvas::OnMaterialChanged. Fow now, only hiding the entity.
        //scene.RemoveEntity(infoBubbleId);
        //infoBubble.placeable.visible = false;
        icon.dynamiccomponent.SetAttribute("infoBubbleVisible", false);

         destPos = currentPos.Add(new float3(0, -cIconMove, 0));
    }

    // Moving of icons disabled for now
    animatedIcons.push(new AnimatedIcon(icon, currentPos, destPos));

    animatedInfoBubbles.push(new AnimatedInfoBubble(icon));

    return true;
}

function ToggleInfoBubbleVisibility(icon)
{
    var infoBubbleVisible = icon.dynamiccomponent.GetAttribute("infoBubbleVisible");
    SetInfoBubbleVisibility(icon, !(infoBubbleVisible != undefined && infoBubbleVisible));
    //if (SetInfoBubbleVisibility(icon, !(infoBubbleVisible != undefined && infoBubbleVisible)))
        //animatedInfoBubbles.push(new AnimatedInfoBubble(icon));
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
    for(i = 0; i < animatedInfoBubbles.length; ++i)
    {
        var e = animatedInfoBubbles[i].icon;
        var infoBubbleVisible = e.dynamiccomponent.GetAttribute("infoBubbleVisible");
        var infoBubbleId = e.dynamiccomponent.GetAttribute("infoBubbleId");
        var infoBubble = scene.EntityById(infoBubbleId);
        var scale = infoBubble.mesh.nodeTransformation.scale.x;
        if (infoBubbleVisible)
        {
            if (!EqualAbs(scale, cVisibleScale, 0.001))
                animatedInfoBubbles[i].t += dt;
            else
            {
                animatedInfoBubbles.splice(i, 1);
                //infoBubble.placeable.visible = infoBubbleVisible;
            }

            if (!EqualAbs(scale, cVisibleScale, 0.001) && animatedInfoBubbles[i].t < cScaleTime)
            {
                scale = Lerp(cHiddenScale, cVisibleScale, animatedInfoBubbles[i].t/cScaleTime);
                infoBubble.mesh.SetAdjustScale(float3.FromScalar(scale));
            }
        }
        else
        {
            if (!EqualAbs(scale, cHiddenScale, 0.001))
                animatedInfoBubbles[i].t += dt;
            else
            {
                animatedInfoBubbles.splice(i, 1);
                //infoBubble.placeable.visible = infoBubbleVisible;
            }

            if (!EqualAbs(scale, cHiddenScale, 0.001) && animatedInfoBubbles[i].t < cScaleTime)
            {
                scale = Lerp(cVisibleScale, cHiddenScale, animatedInfoBubbles[i].t/cScaleTime);
                infoBubble.mesh.SetAdjustScale(float3.FromScalar(scale));
            }
        }
    }

    // Interpolate moving icons
    for(i = 0; i < animatedIcons.length; ++i)
    {
        var currentPos = animatedIcons[i].icon.placeable.WorldPosition();
        var distance = currentPos.Distance(animatedIcons[i].dest);
        if (distance > 1)
        {
            animatedIcons[i].t += dt;
        }
        else
        {
            animatedIcons[i].icon.placeable.SetPosition(animatedIcons[i].dest);
            animatedIcons.splice(i, 1);
        }

        if (animatedIcons[i] && distance > 0.1 && animatedIcons[i].t < cScaleTime)
        {
            currentPos = float3.Lerp(animatedIcons[i].start, animatedIcons[i].dest, animatedIcons[i].t/cScaleTime);
            animatedIcons[i].icon.placeable.SetPosition(currentPos);
        }
    }
}

function Update(dt)
{
    if (!cam)
        cam = renderer.MainCamera();
    if (!cam)
        return;
/*
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
                    if (SetInfoBubbleVisibility(e, true))
                    {
                        autoShownInfoBubbles.push(entityId);
                    }
                }
            }
            else
            {
                if (idx != -1)
                {
                    SetInfoBubbleVisibility(e, false);
                    {
                        autoShownInfoBubbles.splice(idx, 1);
                    }
                }
            }

            // Uncomment to enable auto-rotation and -scaling
            // var dir = e.placeable.WorldPosition().Sub(cam.placeable.WorldPosition()).Normalized();
            // var q = Quat.LookAt(scene.ForwardVector(), dir, scene.UpVector(), scene.UpVector());
            // e.placeable.SetOrientation(q);

            // // Auto-scale
            // e.mesh.SetAdjustScale(float3.FromScalar(DesiredObjectScale(e.mesh)));
        }
    }
*/
    AnimateInfoBubbleScale(dt);
}
