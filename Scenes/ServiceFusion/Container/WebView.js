me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.webkit");
engine.IncludeFile("print.js");

var webView = new QGraphicsWebView();
webView.url = new QUrl("http://m.kaleva.fi");
webView.size = new QSize(me.graphicsviewcanvas.width, me.graphicsviewcanvas.height);
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
