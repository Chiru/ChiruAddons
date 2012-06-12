engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");

function SourceScript(tag, rdfStore)
{
    console.LogInfo("Movie source:");
    console.LogInfo("Tag type: " + tag.type);
    console.LogInfo("Tag data: " + tag.data);
}

function DataScript(tag, rdfStore)
{
    console.LogInfo("Movie data:");
    console.LogInfo("Tag type: " + tag.type);
    console.LogInfo("Tag data: " + tag.data);
}

function CartContainer(parent)
{
    Container.call(this, parent);
    var sourceScript = new Script();
    var dataScript = new Script();
    dataScript.Invoked.connect(DataScript);
    sourceScript.Invoked.connect(SourceScript);
    this.visual.owner.eventManager.RegisterScript(new Tag(RdfVocabulary.sourceApplication, "Movie"), sourceScript);
    this.visual.owner.eventManager.RegisterScript(new Tag(RdfVocabulary.data), dataScript);
    //this.visual.owner.DropToActive(new Tag(RdfVocabulary.sourceApplication, "Movie"), this.visual.owner);
    
}
CartContainer.prototype = new Container();

if (!framework.IsHeadless())
{
    ui.GraphicsView().DropEvent.connect(OnDropEvent);
    //ui.GraphicsView().DragMoveEvent.connect(OnMoveEvent);
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
            cartContainer.visual.HandleMeshDrop(e.source());
        }
    }
}

/*function OnMoveEvent(e)
{
    var data = e.mimeData().data("application/x-hotspot").toString();
    if (data.length > 0)
    {
        var ray = CurrentMouseRay();
        var r = scene.ogre.Raycast(ray, -1);
        if (r.entity && r.entity.id == me.id)
        {
            SetInfoBoubleHightlight(false);
        }
    }
}*/

/*sceneinteract.EntityMouseMove.connect(this, function(entity, button, ray) {
    print(entity.id + " == " + me.id);
    if (entity.id == me.id)
        SetInfoBoubleHightlight(false);
});*/

var cartContainer = new CartContainer(null)