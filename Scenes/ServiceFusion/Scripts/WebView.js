me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.webkit");

me.Action("MousePress").Triggered.connect(function()
{
    var dir = me.placeable.WorldPosition().Sub(renderer.MainCamera().placeable.WorldPosition()).Normalized();
    var q = Quat.LookAt(scene.ForwardVector(), dir, scene.UpVector(), scene.UpVector());
    me.placeable.SetOrientation(q);
});

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
