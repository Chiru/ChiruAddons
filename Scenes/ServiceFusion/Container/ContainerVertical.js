engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

var world = RdfModule.theWorld;

var uiWidget = new VisualContainer(null);
var rootContainer = C3DUiModule.ContainerFactory().CreateContainer(uiWidget);
rootContainer.rdfStore = new RdfMemoryStore(world);
uiWidget.size = new QSize(370, 200);
uiWidget.setLayout(new QVBoxLayout());

var labelVisual = new VisualContainer(uiWidget);
var labelContainer = C3DUiModule.ContainerFactory().CreateContainer(labelVisual, uiWidget);
labelContainer.rdfStore = new RdfMemoryStore(world);
var l1 = new QLabel("VLabel1", labelVisual);
uiWidget.layout().addWidget(labelVisual, 0, 0);

labelVisual = new VisualContainer(uiWidget);
labelContainer = C3DUiModule.ContainerFactory().CreateContainer(labelVisual, uiWidget);
labelContainer.rdfStore = new RdfMemoryStore(world);
var l2 = new QLabel("VLabel2", labelVisual);
uiWidget.layout().addWidget(labelVisual, 0, 0);
 
labelVisual = new VisualContainer(uiWidget);
labelContainer = C3DUiModule.ContainerFactory().CreateContainer(labelVisual, uiWidget);
labelContainer.rdfStore = new RdfMemoryStore(world);
var l3 = new QLabel("VLabel3", labelVisual);
uiWidget.layout().addWidget(labelVisual, 0, 0);
var proxy = me.graphicsviewcanvas.GraphicsScene().addWidget(uiWidget);
uiWidget.show();

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (uiWidget)
        uiWidget.deleteLater();
}