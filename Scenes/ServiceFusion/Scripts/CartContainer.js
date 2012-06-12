engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");

function RunScript(tag, rdfStore)
{
    console.LogInfo("This is a test script!:");
    console.LogInfo("Tag type: " + tag.type);
    console.LogInfo("Tag data: " + tag.data);
    //console.LogInfo("RDF store: " + rdfStore);
}

function CartContainer(parent)
{
    Container.call(this, parent);
    var script = new Script();
    script.Invoked.connect(RunScript);
    this.visual.owner.eventManager.RegisterScript(new Tag(RdfVocabulary.sourceApplication, "Movie"), script);
    this.visual.owner.eventManager.RegisterScript(new Tag(RdfVocabulary.data), script);
    //this.visual.owner.DropToActive(new Tag(RdfVocabulary.sourceApplication, "Movie"), this.visual.owner);
    
}
CartContainer.prototype = new Container();

if (!framework.IsHeadless())
{
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
            cartContainer.visual.HandleMeshDrop(e.source());
        }
    }
}

var cartContainer = new CartContainer(null)