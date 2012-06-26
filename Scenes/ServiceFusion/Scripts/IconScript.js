// IconScript.js - handles animation of 3D icons and info bubbles

// !ref: InfoBubblePrefab.txml

engine.IncludeFile("Log.js");
engine.IncludeFile("MathUtils.js");

var animatedInfoBubbles = [];
//var autoShownInfoBubbles = [];
var animatedIcons = [];
var originalTransforms = {};
var cam = null;

const cInfoBubbleVisibleScale = 14;
const cInfoBubbleHiddenScale = 0.01;
const cIconVerticalMoveFactor = 8.5; // Do not change this value! Alter cInfoBubbleVisibleScale instead.
const cIconScaleFactor = 2.25;
const cIconHorizontalMove = -15;
const cIconVerticalMove = cInfoBubbleVisibleScale * cIconVerticalMoveFactor;
const cIconDepthMove = -10;
const cIconMove = new float3(cIconHorizontalMove, cIconVerticalMove, cIconDepthMove);
const cAnimationTime = 1; // seconds
//const cToggleInfoBubbleVisibilityTreshold = 250 * 250; // Squared distance treshold

// Entry point
if (!framework.IsHeadless())
{
    // Gather info bubbles that want to register themselves to icons
    var infoBubbles = scene.EntitiesWithComponent("EC_DynamicComponent", "Screen");
    for(i in infoBubbles)
    {
        var infoBubble = infoBubbles[i];
        var iconName = infoBubble.dynamiccomponent.GetAttribute("iconName");
        if (iconName)
        {
            var icon = scene.EntityByName(iconName);
            if (!icon)
            {
                LogE("RegisterInfoBubble: icon with name " + iconName + " not found.");
                return;
            }
            var dc = icon.GetComponent("EC_DynamicComponent", "Icon");
            if (!dc)
            {
                LogE("RegisterInfoBubble: dc for icon null.");
                return;
            }

            infoBubble.placeable.SetPosition(icon.placeable.WorldPosition());
            dc.CreateAttribute("uint", "infoBubbleId");
            dc.CreateAttribute("bool", "infoBubbleVisible");
            dc.SetAttribute("infoBubbleId", infoBubble.id);
            dc.SetAttribute("infoBubbleVisible", true);
            // Enforce hide for newly registered info bubbles, do not move icon yet.
            SetInfoBubbleVisibility(icon, false, false);
        }
    }

    sceneinteract.EntityClicked.connect(OnEntityClicked);
    frame.Updated.connect(Update);
}

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return;
    for(i in originalTransforms)
    {
        var icon = scene.EntityById(i);
        if (icon)
        {
            // Reset icon's transform
            icon.placeable.transform = originalTransforms[i];
            // Hide info bubbles
            SetInfoBubbleVisibility(icon, false, false);
            var infoBubbleVisible = icon.dynamiccomponent.GetAttribute("infoBubbleVisible");
            var infoBubbleId = icon.dynamiccomponent.GetAttribute("infoBubbleId");
            var infoBubble = scene.EntityById(infoBubbleId);
            scene.EntityById(infoBubbleId).mesh.SetAdjustScale(float3.FromScalar(cInfoBubbleHiddenScale));
        }
    }
}

function AnimatedIcon(/*Entity*/ icon, /*Transform*/ start, /*Transform*/ dest)
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
function SetInfoBubbleVisibility(/*Entity*/ icon, /*bool*/ visible, /*bool*/ animate)
{
    var infoBubbleId = icon.dynamiccomponent.GetAttribute("infoBubbleId");
    var infoBubbleVisible = icon.dynamiccomponent.GetAttribute("infoBubbleVisible");
    //if (visible == infoBubbleVisible)
      //  return false;

    if (animate)
        var currentTr = icon.placeable.transform, destTr = icon.placeable.transform;

    if (visible) // show
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
        
        if (animate)
        {
            originalTransforms[icon.id] = currentTr;
            destTr.pos = destTr.pos.Add(cIconMove);
            destTr.rot.y = 0;
            destTr.scale = destTr.scale.Mul(cIconScaleFactor);
        }
    }
    else // hide
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
        
        if (animate)
        {
            destTr.pos = destTr.pos.Add(cIconMove.Mul(-1));
            destTr.rot = originalTransforms[icon.id].rot;
            destTr.scale = originalTransforms[icon.id].scale;
        }
    }

    if (animate)
    {
        animatedIcons.push(new AnimatedIcon(icon, currentTr, destTr));
        animatedInfoBubbles.push(new AnimatedInfoBubble(icon));
    }
    else
    {
        var dest = visible ? cInfoBubbleVisibleScale : cInfoBubbleHiddenScale;
        infoBubble.mesh.SetAdjustScale(float3.FromScalar(dest));
    }

    return true;
}

function ToggleInfoBubbleVisibility(icon)
{
    var infoBubbleVisible = icon.dynamiccomponent.GetAttribute("infoBubbleVisible");
    SetInfoBubbleVisibility(icon, !(infoBubbleVisible != undefined && infoBubbleVisible), true);
}

/*
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
*/

function AnimateInfoBubbles(dt)
{
    for(i = 0; i < animatedInfoBubbles.length; ++i)
    {
        var e = animatedInfoBubbles[i].icon;
        var infoBubbleVisible = e.dynamiccomponent.GetAttribute("infoBubbleVisible");
        var infoBubbleId = e.dynamiccomponent.GetAttribute("infoBubbleId");
        var infoBubble = scene.EntityById(infoBubbleId);
        var start = infoBubbleVisible ? cInfoBubbleHiddenScale : cInfoBubbleVisibleScale;
        var dest = infoBubbleVisible ? cInfoBubbleVisibleScale : cInfoBubbleHiddenScale;

        animatedInfoBubbles[i].t += dt;
        if (animatedInfoBubbles[i].t < cAnimationTime)
        {
            scale = Lerp(start, dest, animatedInfoBubbles[i].t/cAnimationTime);
            infoBubble.mesh.SetAdjustScale(float3.FromScalar(scale));
        }
        else
        {
            infoBubble.mesh.SetAdjustScale(float3.FromScalar(dest));
            animatedInfoBubbles.splice(i, 1);
        }
    }
}

function AnimateIcons(dt)
{
    for(i = 0; i < animatedIcons.length; ++i)
    {
        animatedIcons[i].t += dt;
        if (animatedIcons[i].t < cAnimationTime)
        {
            var t = animatedIcons[i].icon.placeable.transform;
            t.pos = float3.Lerp(animatedIcons[i].start.pos, animatedIcons[i].dest.pos, animatedIcons[i].t/cAnimationTime);
            t.rot = float3.Lerp(animatedIcons[i].start.rot, animatedIcons[i].dest.rot, animatedIcons[i].t/cAnimationTime);
            t.scale = float3.Lerp(animatedIcons[i].start.scale, animatedIcons[i].dest.scale, animatedIcons[i].t/cAnimationTime);
            animatedIcons[i].icon.placeable.transform = t;
        }
        else
        {
            animatedIcons[i].icon.placeable.transform = animatedIcons[i].dest;
            animatedIcons.splice(i, 1);
        }
    }
}

function Update(dt)
{
    if (!cam)
        cam = renderer.MainCamera();
    if (!cam)
        return;
    AnimateInfoBubbles(dt);
    AnimateIcons(dt);
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
}
