// !ref: ContainerVertical.ui

me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

var scrollArea = new QScrollArea();
scrollArea.size = new QSize(200, 200);
scrollArea.horizontalScrollBarPolicy = Qt.ScrollBarAlwaysOn;
scrollArea.verticalScrollBarPolicy = Qt.ScrollBarAlwaysOn

// Instantiate the window. The window will be emdedded to the main window as a root widget (no parent).
//var uiWidget = asset.GetAsset("ContainerVertical.ui").Instantiate(false, 0);
var uiWidget = new DragDropWidget(null);
uiWidget.size = new QSize(200, 200);
uiWidget.setLayout(new QVBoxLayout());
uiWidget.layout().addWidget(new QLabel("VLabel1", uiWidget), 0, 0);
uiWidget.layout().addWidget(new QLabel("VLabel2", uiWidget), 0, 0);
uiWidget.layout().addWidget(new QLabel("VLabel3", uiWidget), 0, 0);
var proxy = me.graphicsviewcanvas.GraphicsScene().addWidget(scrollArea/*uiWidget*/);
uiWidget.show();
scrollArea.setWidget(uiWidget);

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (uiWidget)
        uiWidget.deleteLater();
}
