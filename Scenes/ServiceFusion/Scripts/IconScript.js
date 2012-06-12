// IconScript.js - handles behavior of in-world 3D icons
// - automatic scaling,
// - rotating towards camera,
// - and showing of info bubbles

// !ref: InfoBubblePrefab.txml

engine.IncludeFile("Log.js");

// TODO
//const showInfoTreshold = 20;

sceneinteract.EntityClicked.connect(OnEntityClicked);

function IsEntityAnIcon(e)
{
    return e.placeable != null && e.mesh != null && e.dynamiccomponent != null && e.dynamiccomponent.name == "Icon";
}

function ShowInfoBubble(entity)
{
    var infoBubbleVisible = entity.dynamiccomponent.GetAttribute("infoBubbleVisible");
    if (infoBubbleVisible != undefined && infoBubbleVisible) // Info bubble visible, remove it.
    {
        var infoBubbleId = entity.dynamiccomponent.GetAttribute("infoBubbleId");
        var infoBubble = scene.EntityById(infoBubbleId);
        if (!infoBubble)
        {
            LogE("Trying to remove non-existing info bubble with entity ID " + infoBubbleId);
            return;
        }
        // TODO: crashes to EC_GraphicsViewCanvas::OnMaterialChanged. Fow now, only hiding the entity.
        //scene.RemoveEntity(infoBubbleId);
        infoBubble.placeable.visible = false;
        entity.dynamiccomponent.SetAttribute("infoBubbleVisible", infoBubble.placeable.visible);
    }
    else
    {
        var infoBubbleId = entity.dynamiccomponent.GetAttribute("infoBubbleId");
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
            infoBubble.placeable.SetPosition(entity.placeable.WorldPosition());
            infoBubble.placeable.SetScale(2,2,2);
            /*
            var t = infoBubble.placeable.transform;
            t.rot = new float3(0, 180, 0);
            t.pos = entity.placeable.WorldPosition();
            infoBubble.placeable.transform = t;
            */
            entity.dynamiccomponent.CreateAttribute("bool", "infoBubbleVisible");
            entity.dynamiccomponent.SetAttribute("infoBubbleVisible", infoBubble.placeable.visible);
            entity.dynamiccomponent.CreateAttribute("uint", "infoBubbleId");
            entity.dynamiccomponent.SetAttribute("infoBubbleId", infoBubble.id);
        }
        else
        {
            var infoBubble = scene.EntityById(infoBubbleId);
            if (!infoBubble)
            {
                LogE("Trying to set visibility of a non-existing info bubble with entity ID "+ infoBubbleId);
                return;
            }
            infoBubble.placeable.visible = true;
            entity.dynamiccomponent.SetAttribute("infoBubbleVisible", infoBubble.placeable.visible);
        }
    }
}

function OnEntityClicked(entity)
{
    if (IsEntityAnIcon(entity))
        ShowInfoBubble(entity);
}

frame.Updated.connect(Update)
var cam = null;

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

function Update()
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
        // Auto-rotate and -scale disabled for now.
/*
            var dir = e.placeable.WorldPosition().Sub(cam.placeable.WorldPosition()).Normalized();
            var q = Quat.LookAt(scene.ForwardVector(), dir, scene.UpVector(), scene.UpVector());
            e.placeable.SetOrientation(q);

            // Auto-scale
            e.mesh.SetAdjustScale(float3.FromScalar(DesiredObjectScale(e.mesh)));
*/

            // TODO
//            if (e.placeable.WorldPosition().Distance(cam.placeable.WorldPosition()) < showInfoTreshold)
//                ShowInfoBubble(e);
        }
    }
}
