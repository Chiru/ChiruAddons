engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("VisualContainerUtils.js");

var pp = scene.GetEntityByName("powerPoint");
var ppOrigLocation = pp.placeable.transform;

if (!framework.IsHeadless())
{
    //ui.GraphicsView().DragMoveEvent.connect(OnDropEvent);
    ui.GraphicsView().DropEvent.connect(OnDropEvent);
}

function OnDropEvent(e)
{
    var data = e.mimeData().data("application/x-hotspot").toString();
    if (data.length > 0)
    {
        var ray = CurrentMouseRay();
        var r = scene.ogre.Raycast(ray, -1);

        if (r.entity && r.entity.id == me.id)
        {
            var slide = findChild(e.source(), "chiru_presentation");
            if(slide)
                setPresentation(slide.property("slides"));
        }
    }
}

function setPresentation(slideMat)
{

    var transferPtr = asset.RequestAsset(slideMat, "OgreMaterial", true);
    transferPtr.Succeeded.connect(function(a) {

        if(a.IsLoaded())
            me.mesh.SetMaterial(0,slideMat);       
    });

    transferPtr.Failed.connect(function(a, r) {});
   
    pp.placeable.transform = ppOrigLocation;
}
