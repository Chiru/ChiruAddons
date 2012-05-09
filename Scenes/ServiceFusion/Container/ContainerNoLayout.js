// !ref: ContainerNoLayout.ui

me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

ui.MainWindow().mouseTracking = true;
ui.GraphicsView().mouseTracking = true;

var uiWidget = new DragDropWidget(null);
uiWidget.size = new QSize(370, 200);
var label = new QLabel("FreeLabel", uiWidget);
label.font = new QFont("Arial", 24);
label.setAttribute(Qt.WA_DeleteOnClose);
//var uiWidget = asset.GetAsset("ContainerNoLayout.ui").Instantiate(false, 0);
var proxy = me.graphicsviewcanvas.GraphicsScene().addWidget(uiWidget);
//me.graphicsviewcanvas.height = uiWidget.height;
//me.graphicsviewcanvas.width = uiWidget.width
uiWidget.show();

var uiWidget2 = null; //new DragDropWidget(null);
/*
//uiWidget2.setParent(ui.MainWindow());
//uiWidget2.windowFlags = Qt.Tool;
uiWidget2.windowTitle = "No Layout";
uiWidget2.show();
*/
function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (uiWidget)
        uiWidget.deleteLater();
    if (uiWidget2)
        uiWidget2.deleteLater();
}
