engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

ui.MainWindow().mouseTracking = true;
ui.GraphicsView().mouseTracking = true;

var world = RdfModule.theWorld;

var uiWidget = new VisualContainer(null);
var rootContainer = C3DUiModule.ContainerFactory().CreateContainer(uiWidget);
rootContainer.rdfStore = new RdfMemoryStore(world);
uiWidget.size = new QSize(370, 200);

var labelVisual = new VisualContainer(uiWidget);
labelVisual.move(100,100);
var labelContainer = C3DUiModule.ContainerFactory().CreateContainer(labelVisual, uiWidget);
labelContainer.rdfStore = new RdfMemoryStore(world);
var label = new QLabel("FreeLabel", labelVisual);
label.font = new QFont("Arial", 24);
label.setAttribute(Qt.WA_DeleteOnClose);
labelVisual.size = label.sizeHint;
print(me.GetComponent("EC_GraphicsViewCanvas", "Content"));
me.GetComponent("EC_GraphicsViewCanvas", "Content").GraphicsScene().addWidget(uiWidget);
//var proxy = me.graphicsviewcanvas.GraphicsScene().addWidget(uiWidget);
uiWidget.show();

var titleWidget = new QLabel("NoLayout");
titleWidget.font = new QFont("Arial", 24);
me.GetComponent("EC_GraphicsViewCanvas", "Title").GraphicsScene().addWidget(titleWidget);
titleWidget.show();

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (uiWidget)
        uiWidget.deleteLater();
    if (titleWidget)
        titleWidget.deleteLater();
}
