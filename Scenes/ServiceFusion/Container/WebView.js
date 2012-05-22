me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.webkit");

var webView = new QGraphicsWebView();
webView.url = new QUrl("http://m.kaleva.fi");
webView.pos = new QPoint(20, 20);
webView.size = new QSize(me.graphicsviewcanvas.width - 30, me.graphicsviewcanvas.height - 20);
me.graphicsviewcanvas.GraphicsScene().addItem(webView);

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (webView)
    {
        webView.deleteLater();
        webView = null;
    }
}
