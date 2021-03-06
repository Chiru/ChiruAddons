me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.webkit");
engine.IncludeFile("Localisation.js");
engine.IncludeFile("WebViewInterface.js");

const cAnimationTime = 1; // seconds
var startOrientation, destOrientation;
var DefaultUrlAddress = NEWS_URL;
var gvc = me.graphicsviewcanvas;

me.Action("MousePress").Triggered.connect(function()
{
    startOrientation = me.placeable.WorldOrientation();
    var dir = me.placeable.WorldPosition().Sub(renderer.MainCamera().placeable.WorldPosition()).Normalized();
    destOrientation = Quat.LookAt(scene.ForwardVector(), dir, scene.UpVector(), scene.UpVector());
});

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

setURL(DefaultUrlAddress);
