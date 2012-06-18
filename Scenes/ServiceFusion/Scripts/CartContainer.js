engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");

var cartContainer = null;

function ValidateData(rdfData)
{
    Select();
    return false;
}

var supportedTypes = {"Movie":1};

function SourceScript(tag, rdfStore)
{
    if (cartContainer && supportedTypes[tag.data])
    {
        childContainer = new BaseContainer(cartContainer.visual);
        childContainer.container.rdfStore.FromString(rdfStore.toString());
        print(childContainer.container.rdfStore);
    }
}

function CartContainer(parent)
{
    BaseContainer.call(this, parent);
    var sourceScript = new Script();
    sourceScript.Invoked.connect(SourceScript);
    this.visual.owner.eventManager.RegisterScript(new Tag(RdfVocabulary.sourceApplication, "Movie"), sourceScript);
}
CartContainer.prototype = new BaseContainer();

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

cartContainer = new CartContainer(null)

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (cartContainer)
    {
        cartContainer.visual.deleteLater();
        cartContainer = null;
    }
}