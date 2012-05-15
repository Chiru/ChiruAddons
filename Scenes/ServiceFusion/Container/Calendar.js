me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

var calendarWidget = new QCalendarWidget();
calendarWidget.size = new QSize(me.graphicsviewcanvas.width, me.graphicsviewcanvas.height);
calendarWidget.gridVisible = true;
me.graphicsviewcanvas.GraphicsScene().addWidget(calendarWidget);
calendarWidget.show();

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (calendarWidget)
    {
        calendarWidget.deleteLater();
        calendarWidget = null;
    }
}
