me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

var world = RdfModule.theWorld;

var uiWidget = new VisualContainer(null);
var rootContainer = C3DUiModule.ContainerFactory().CreateContainer(uiWidget);
rootContainer.rdfStore = new RdfMemoryStore(world);
var content = me.GetComponent("EC_GraphicsViewCanvas", "Content")
uiWidget.size = new QSize(content.width, content.height);
uiWidget.setLayout(new QHBoxLayout());

var labelVisual = new VisualContainer(uiWidget);
var labelContainer = C3DUiModule.ContainerFactory().CreateContainer(labelVisual, uiWidget);
labelContainer.rdfStore = new RdfMemoryStore(world);
var l1 = new QLabel("HLabel1", labelVisual);
uiWidget.layout().addWidget(labelVisual, 0, 0);

labelVisual = new VisualContainer(uiWidget);
labelContainer = C3DUiModule.ContainerFactory().CreateContainer(labelVisual, uiWidget);
labelContainer.rdfStore = new RdfMemoryStore(world);
var l2 = new QLabel("HLabel2", labelVisual);
uiWidget.layout().addWidget(labelVisual, 0, 0);

labelVisual = new VisualContainer(uiWidget);
labelContainer = C3DUiModule.ContainerFactory().CreateContainer(labelVisual, uiWidget);
labelContainer.rdfStore = new RdfMemoryStore(world);
var l3 = new QLabel("HLabel3", labelVisual);
uiWidget.layout().addWidget(labelVisual, 0, 0);

content.GraphicsScene().addWidget(uiWidget);
uiWidget.show();

var titleWidget = new QLabel("Horizontal");
titleWidget.font = new QFont("Arial", 24);
me.GetComponent("EC_GraphicsViewCanvas", "Title").GraphicsScene().addWidget(titleWidget);
titleWidget.show();

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (uiWidget)
    {
        uiWidget.deleteLater();
        uiWidget = null;
    }
    if (titleWidget)
    {
        titleWidget.deleteLater();
        titleWidget = null;
    }
}