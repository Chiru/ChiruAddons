// !ref: ContainerHorizontal.ui

me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

// Instantiate the window. The window will be emdedded to the main window as a root widget (no parent).
//var uiWidget = asset.GetAsset("ContainerHorizontal.ui").Instantiate(false, 0);
var uiWidget = new DragDropWidget(null);
uiWidget.size = new QSize(370, 200);
uiWidget.setLayout(new QHBoxLayout());
uiWidget.layout().addWidget(new QLabel("HLabel1", uiWidget), 0, 0);
uiWidget.layout().addWidget(new QLabel("HLabel2", uiWidget), 0, 0);
uiWidget.layout().addWidget(new QLabel("HLabel3", uiWidget), 0, 0);
var proxy = me.graphicsviewcanvas.GraphicsScene().addWidget(uiWidget);
uiWidget.show();

var uiWidget2 = null; //new DragDropWidget(null);
/*
//uiWidget2.setParent(ui.MainWindow());
//uiWidget2.windowFlags |= Qt.Tool;
uiWidget2.windowTitle = "Horizontal";
uiWidget2.setLayout(new QHBoxLayout());
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
