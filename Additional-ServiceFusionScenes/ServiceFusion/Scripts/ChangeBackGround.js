engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

var ic = input.RegisterInputContextRaw("ChangeBackGround", 90);
ic.KeyEventReceived.connect(HandleKeyEvent);
var original = "true";

function HandleKeyEvent()
{
    if(input.IsKeyDown(Qt.Key_B))
    {
        var b = scene.EntityByName("background");
        var b2 = scene.EntityByName("background2");

        if(original == "true")
        {
            b.placeable.visible = false;
            b2.placeable.visible = true;
            original = "false";
        }
        else
        {
            b.placeable.visible = true;
            b2.placeable.visible = false;
            original = "true";
        }
        //b.mesh.SetMaterial(0, "valkonen.material")
    }
}
