engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");

me.Action("MousePress").Triggered.connect(MousePressed);

var cartContainer = null;
var cartVisualEntity = scene.GetEntityByName("cart_item");

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
        if (cartVisualEntity)
            cartVisualEntity.placeable.visible = true;
        me.Exec(1, "CartItemAdded", tag.data, rdfStore.toString());
    }
}

me.Action("CartItemAdded").Triggered.connect(function(type, rdfStoreData) { print(rdfStoreData); });

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

function MousePressed()
{
    if (!scene.EntityByName("MovieLoginDialog"))
    {
        var movieLoginEntity = scene.CreateEntity(scene.NextFreeId(), ["EC_Script", "EC_Name"]);
        movieLoginEntity.SetName("MovieLoginDialog");
        var script = movieLoginEntity.GetOrCreateComponent("EC_Script");
        script.scriptRef = new AssetReference("MovieLogin.js");
        script.runOnLoad = true;
    }

    if (!scene.EntityByName("MovieSeatDialog"))
    {
        var movieSeatEntity = scene.CreateEntity(scene.NextFreeId(), ["EC_Script", "EC_Name"]);
        movieSeatEntity.SetName("MovieSeatDialog");
        var script = movieSeatEntity.GetOrCreateComponent("EC_Script");
        script.scriptRef = new AssetReference("MovieSeatSelection.js");
        script.runOnLoad = true;
    }

    if (!scene.EntityByName("MoviePaymentDialog"))
    {
        var moviePaymentEntity = scene.CreateEntity(scene.NextFreeId(), ["EC_Script", "EC_Name"]);
        moviePaymentEntity.SetName("MoviePaymentDialog");
        var script = moviePaymentEntity.GetOrCreateComponent("EC_Script");
        script.scriptRef = new AssetReference("MoviePayment.js");
        script.runOnLoad = true;
    }
}

cartContainer = new CartContainer(null);

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
