me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.webkit");

const cAnimationTime = 1; // seconds
var startOrientation, destOrientation;

me.Action("MousePress").Triggered.connect(function()
{
    startOrientation = me.placeable.WorldOrientation();
    var dir = me.placeable.WorldPosition().Sub(renderer.MainCamera().placeable.WorldPosition()).Normalized();
    destOrientation = Quat.LookAt(scene.ForwardVector(), dir, scene.UpVector(), scene.UpVector());

    timer = 0;
    frame.Updated.connect(Animate);
});

var timer = 0;
function Animate(dt)
{
    timer += dt;
    if (timer < cAnimationTime)
    {
        var t = me.placeable.transform;
        var q = Quat.Slerp(startOrientation, destOrientation, timer/cAnimationTime);
        t.SetOrientation(q);
        t.rot.y = 0;
        me.placeable.transform = t;
    }
    else
        frame.Updated.disconnect(Animate);
}

var webView = new QGraphicsWebView();
webView.url = new QUrl("http://chiru.cie.fi/servicefusion/OuluLive.html");
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
