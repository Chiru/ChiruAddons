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

function CartItem(type, data)
{
    this.type = type;
    this.data = data;
}

var supportedTypes = {"Movie":1};
var cartItems = new Array();
function SourceScript(tag, rdfStore)
{
    if (cartContainer && supportedTypes[tag.data])
    {
        childContainer = new BaseContainer(cartContainer.visual);
        childContainer.container.rdfStore.FromString(rdfStore.toString());
        if (cartVisualEntity)
            cartVisualEntity.placeable.visible = true;
        
        cartItems.push(FetchMovieData(childContainer.container.rdfStore));
    }
}

function FetchMovieData(rdfStore)
{
    var statements = Select(rdfStore, null, RdfVocabulary.data, null);
    if (statements.length == 3)
    {
        var item = new CartItem("Movie", new Object);
        item.data["time"] = statements[0].object.literal;
        item.data["title"] = statements[1].object.literal;
        item.data["auditorium"] = statements[2].object.literal;
        return item;
    }
    FreeStatements(statements)
}

function GetMovieCartItem()
{
    for(var i = 0; i < cartItems.length; ++i)
        if (cartItems[i].type == "Movie")
            return cartItems[i];
    return null;
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

function SendMovieData()
{
    var movieItem = GetMovieCartItem();
    var movieLoginEntity = scene.GetEntityByName("MovieLoginDialog");
    var moviePayment = scene.GetEntityByName("MoviePaymentDialog");
    if (!movieItem || !movieLoginEntity || !moviePayment)
        return;

    var date = new Date(movieItem.data["time"]);
    var hour = date.getHours();
    var minute = date.getMinutes();
    if (minute == 0) minute = "00";
    var date = date.getDate() + "." + (date.getMonth() + 1) + "." + date.getFullYear();
    var params = [movieItem.data["title"], movieItem.data["auditorium"], (hour + ":" + minute), date.toString()];
    movieLoginEntity["Exec(EntityAction::ExecTypeField,QString,QVariantList)"](1, "SetMovieInfo", params);
    moviePayment["Exec(EntityAction::ExecTypeField,QString,QVariantList)"](1, "SetMovieInfo", params);
}

function MousePressed()
{
    var movieItem = GetMovieCartItem();
    if (!movieItem)
        return;

    if (!scene.EntityByName("MovieLoginDialog"))
    {
        var movieLoginEntity = scene.CreateEntity(scene.NextFreeId(), ["EC_Script", "EC_Name"]);
        movieLoginEntity.SetName("MovieLoginDialog");
        var script = movieLoginEntity.GetOrCreateComponent("EC_Script");
        script.scriptRef = new AssetReference("MovieLogin.js");
        script.runOnLoad = true;
        movieLoginEntity.Action("RequestMovieInfo").Triggered.connect(SendMovieData);
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
    
    frame.DelayedExecute(1.0).Triggered.connect(SendMovieData);
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
