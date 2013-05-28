engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("VisualContainerUtils.js");

sceneinteract.EntityClicked.connect(EntityClicked)

var ppContainer = null;
var ppContainerCreated = false;

function EntityClicked(entity)
{
    if (entity.id == me.id)
    {
        if(!ppContainerCreated)
        {
            ppContainer = new BaseContainer();
            var ppw = new QWidget();
            ppw.objectName = "chiru_presentation";
            ppw.setProperty("slides", "local://chiruSlide.material");
            ppw.setParent(ppContainer.visual);
            ppContainerCreated = true;
            ppContainer.visual.StartDrag(new QPoint(0,0));
        }
        else
            ppContainer.visual.StartDrag(new QPoint(0,0));
    }
}
