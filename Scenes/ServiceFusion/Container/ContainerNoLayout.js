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
var proxy = me.graphicsviewcanvas.GraphicsScene().addWidget(uiWidget);
uiWidget.show();

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (uiWidget)
        uiWidget.deleteLater();
}
