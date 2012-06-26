
var defaultTransform = me.placeable.transform;

me.Action("Reset").Triggered.connect(function()
{
    me.placeable.transform = defaultTransform;
});

var show_debug = false;

function debug(arg) {
    if (show_debug)
        print(arg)
}

engine.IncludeFile("VisualContainerUtils.js");
engine.IncludeFile("RdfVocabulary.js");

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

debug("Radio.js loading");
var dummyVc = new VisualContainer(null);
var dummyContainer = new Container(dummyVc);

var script = new Script();

script.Invoked.connect(RunScript);

dummyContainer.eventManager.RegisterScript(new Tag(RdfVocabulary.sourceApplication, "Song"), script);
dummyContainer.eventManager.RegisterScript(new Tag("test", "test"), script);

function RunScript(tag, rdfStore)
{
    debug("RunScript called");
    debug("This is a test script!:");
    debug("Tag type: " + tag.type);
    debug("Tag data: " + tag.data);
    debug("RDF store: " + rdfStore.toString());

    var lyricsent = scene.GetEntityByName("LyricsContainer");
    debug("got entity");
    lyricsent.Exec(1, "ShowLyrics");
    debug("sent action");
}

if (!framework.IsHeadless())
{
    ui.GraphicsView().DropEvent.connect(OnDropEvent);
    //ui.GraphicsView().DragMoveEvent.connect(OnMoveEvent);
}

function OnDropEvent(e)
{
    var data = e.mimeData().data("application/x-hotspot").toString();
    print(data.length);
    if (data.length > 0)
    {
        var ray = CurrentMouseRay();
        var r = scene.ogre.Raycast(ray, -1);
        if (r.entity && r.entity.id == me.id)
        {
            debug("radio: call handlemeshdrop");
            dummyContainer.visual.HandleMeshDrop(e.source());
        }
    }
}

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (dummyVc)
    {
        dummyVc.deleteLater();
        dummyVc = null;
    }
}
