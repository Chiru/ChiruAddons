me.Action("Show").Triggered.connect(DimBackground);
me.Action("Hide").Triggered.connect(ClearBackground);
me.Action("Move").Triggered.connect(SetPosition);

function CreateEntity()
{
    var mesh = me.GetOrCreateComponent("EC_Mesh");

    var placeable = me.GetOrCreateComponent("EC_Placeable");
    var transform = placeable.transform;
    transform.pos = float3(0, 123, -324);
    transform.rot = float3(-62, 0, 0);
    transform.scale = float3(200, 1, 150);
    placeable.transform = transform;
    placeable.visible = false;

    mesh.meshRef = "Plane.mesh";
    var materials = mesh.meshMaterial;
    materials[0] = "BlackTransparent.material";
    mesh.meshMaterial = materials;

    SetPosition();
}

function SetPosition()
{
    var camera = scene.GetEntityByName("UiCamera");
    if(camera)
    {
        var camPos = camera.placeable.transform.pos;
        var pos = camPos.Sub(new float3(0, 130, -250)); // Camera's orientation stays constant
        var transform = me.placeable.transform;
        transform.pos = pos;
        me.placeable.transform = transform;
    }
    else
        print("No camera");
}

function DimBackground()
{
    print("Dimming background");
    me.placeable.visible = true;
}

function ClearBackground()
{
    print("Clearing background");
    me.placeable.visible = false;
}

CreateEntity();
