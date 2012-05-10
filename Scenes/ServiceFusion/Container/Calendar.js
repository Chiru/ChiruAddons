me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

var calendarWidget = new QCalendarWidget();
calendarWidget.gridVisible = true;
me.graphicsviewcanvas.GraphicsScene().addWidget(calendarWidget);
calendarWidget.show();

//calendarWidget.pos = new QPoint(70,-300);
frame.DelayedExecute(2).Triggered.connect(RepositionWidget);

function RepositionWidget()
{
    if (calendarWidget)
        calendarWidget.pos = new QPoint(65,-300);
}

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
