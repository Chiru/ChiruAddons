me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

/* 
tbd. try drag'n'drop info passing by putting entity id in layout widget property
*/

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.network");
engine.IncludeFile("VisualContainerUtils.js");

// LookAtCamera START
const cAnimationTime = 1; // seconds
var startOrientation, destOrientation;

var pressCount = 0;
var widgets = null;

var drop_callback = null;

me.Action("MousePress").Triggered.connect(function()
{
    if (++pressCount % 2 == 0)
    {
        me.placeable.visible = false;
        return;
    }

    startOrientation = me.placeable.WorldOrientation();
    var dir = me.placeable.WorldPosition().Sub(renderer.MainCamera().placeable.WorldPosition()).Normalized();
    destOrientation = Quat.LookAt(scene.ForwardVector(), dir, scene.UpVector(), scene.UpVector());

    timer = 0;
    frame.Updated.connect(Animate);
});

var timer = 0;
function Animate(dt)
{
    timer += dt;
    if (timer < cAnimationTime)
    {
        var t = me.placeable.transform;
        var q = Quat.Slerp(startOrientation, destOrientation, timer/cAnimationTime);
        t.SetOrientation(q);
        t.rot.y = 0;
        me.placeable.transform = t;
    }
    else
        frame.Updated.disconnect(Animate);
}
// LookAtCamera END

me.placeable.visible = true;

function FeedContainer(parent)
{
    print("container init");
    BaseContainer.call(this, parent);
    this.visual.styleSheet = "background-color:white;";
    this.visual.size = new QSize(512, 512);
    var contentCanvas = me.GetComponent("EC_GraphicsViewCanvas", "Content");
    contentCanvas.styleSheet = "background-color:white;";
    contentCanvas.width = this.visual.width;
    contentCanvas.height = this.visual.height;
    this.visual.setLayout(new QVBoxLayout());
    this.visual.layout().setContentsMargins(0, 0, 0, 0);
    this.visual.layout().setSpacing(0);

    var main = new QWidget();
    main.styleSheet = "background-color:white;";
    main.setLayout(new QVBoxLayout());
    main.layout().setContentsMargins(10, 0, 0, 0); 
    main.objectName = "MainWidget";
    main.setProperty("entity_name", me.name)
    this.main_widget = main;
    var vcontainer = CreateVisualContainer(main, new QHBoxLayout(), this.visual);

    this.titleLabel = new QLabel();
    this.titleLabel.styleSheet = "background-color:white;";
    this.titleLabel.font = new QFont("FreeSans", 24);
    this.titleLabel.alignment = 0x0004; // Qt::AlignHCenter
    this.titleLabel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preffered);
    main.layout().addWidget(this.titleLabel, null, null);

    this.label2 = new QLabel("label2");
    this.label2.styleSheet = "background-color:white;";
    this.label2.font = new QFont("FreeSans", 18);
    this.label2.alignment = 0x0004; // Qt::AlignHCenter
    this.label2.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preffered);
    main.layout().addWidget(this.label2, null, null);

    this.bodylabel = new QLabel("bodylabel");
    this.bodylabel.styleSheet = "background-color:white;";
    this.bodylabel.font = new QFont("FreeSans", 12);
    this.bodylabel.wordWrap = true;
    this.bodylabel.setSizePolicy(QSizePolicy.Preffered, QSizePolicy.Preffered);
//    this.bodylabel.setSizePolicy (QSizePolicy.Expanding, QSizePolicy.Preffered);  -Stinkfist
    main.layout().addWidget(this.bodylabel, null, null);

    vcontainer.layout().addWidget(main, null, null);
    vcontainer.layout().spacing = 0;

    widgets = this;
    me.Action("ServiceInfo").Triggered.connect(GotNews);
}

FeedContainer.prototype = new BaseContainer();

function CreateContainer(entity)
{
    var container = new FeedContainer(null)
    entity.graphicsviewcanvas.GraphicsScene().addWidget(container.visual);
    return container;
}

var container = CreateContainer(me);

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (container)
    {
        container.visual.deleteLater();
        container = null;
    }
}

function OnDropEvent(e)
{
    if (!drop_callback)
	return;

    var data = e.mimeData().data("application/x-hotspot").toString();
    if (data.length <= 0)
	return;

    var ray = CurrentMouseRay();
    var r = scene.ogre.Raycast(ray, -1);
    if (! (r.entity && r.entity.id == me.id)) {
	print("drop not for this entity")
	return;
    }
    // var searchWord = findChild(e.source(), "entity_id")
    // if (searchWord)
    // {
    //     DoSearch(searchWord.property("searchWord"), searchWord.property("searchType"));
    // }
    
    var main_widget = findChild(e.source(), "MainWidget");
    if (!main_widget) {
	print("main widget not found on drop event");
	return;
    }
    var entity_name = main_widget.property("entity_name");
    if (entity_name === null || entity_name === undefined) {
	print("no property entity_name in source");
	return;
    }
    print("got entity name: " + entity_name + " type " + typeof(entity_name));
    ent = scene.GetEntityByName(entity_name)
    if (ent)
	drop_callback(ent);
}

function SetDropHandler(fun) {
    if (framework.IsHeadless()) {
	print("headless mode: not registering drop handler");
	return;
    }
    ui.GraphicsView().DropEvent.connect(OnDropEvent);
    drop_callback = fun;
}

function SetInfoHandler(fun) {
    me.Action("ServiceInfo").Triggered.connect(GotNews);
}
