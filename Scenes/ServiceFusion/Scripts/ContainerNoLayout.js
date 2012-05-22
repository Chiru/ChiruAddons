me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

ui.MainWindow().mouseTracking = true;
ui.GraphicsView().mouseTracking = true;

var world = RdfModule.theWorld;

var uiWidget = new VisualContainer(null);
var rootContainer = C3DUiModule.ContainerFactory().CreateContainer(uiWidget);
rootContainer.rdfStore = new RdfMemoryStore(world);
var content = me.GetComponent("EC_GraphicsViewCanvas", "Content");
uiWidget.size = new QSize(content.width, content.height);

var labelVisual = new VisualContainer(uiWidget);
labelVisual.move(100,100);
var labelContainer = C3DUiModule.ContainerFactory().CreateContainer(labelVisual, uiWidget);
labelContainer.rdfStore = new RdfMemoryStore(world);
var label = new QLabel("FreeLabel", labelVisual);
label.font = new QFont("Arial", 24);
label.setAttribute(Qt.WA_DeleteOnClose);
labelVisual.size = label.sizeHint;
content.GraphicsScene().addWidget(uiWidget);
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
