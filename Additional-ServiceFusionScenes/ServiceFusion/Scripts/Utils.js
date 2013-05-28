// Utils.js - Misc. utility functions
// Prerequisities:
// engine.ImportExtension("qt.core");
// engine.IncludeFile("MathUtils.js");

// QByteArray's toString not exposed in QtScript, must do it manually.
QByteArray.prototype.toString = function()
{
    ts = new QTextStream(this, QIODevice.ReadOnly);
    ts.setCodec("UTF-8");
    return ts.readAll();
}

// From http://stackoverflow.org/wiki/JavaScript_string_trim
String.prototype.trim = function() {
    return this.replace(/^\s+|\s+$/g,"");
}
String.prototype.ltrim = function() {
    return this.replace(/^\s+/,"");
}
String.prototype.rtrim = function() {
    return this.replace(/\s+$/,"");
}

// Returns mouse ray from the active camera at screen point coordinates.
function MouseRay(x, y)
{
    return renderer.MainCameraComponent().GetMouseRay(x/ui.GraphicsScene().width(), y/ui.GraphicsScene().height());
}

// TODO: Is this needed?
function IsTouchStateMoved(touch)
{
    return touch.state() == Qt.TouchPointMoved && !EqualsQPointF(touch.pos(), touch.lastPos());
}

// For some reason at least on my Win7 Fujitsu T901 state of a touch event is never stationary.
// As a workaround, compare pos and lastPos of touch even manually to see if we have a stationary
// touch or not.
// TODO: check out what's the case on Ubuntu.
function IsTouchStateStationary(touch)
{
    return touch.state() == Qt.TouchPointStationary || EqualsQPointF(touch.pos(), touch.lastPos());
}

// Checks whether or not we're allowed to move an object in the scene.
function IsObjectMovable(e)
{
    return e.placeable && !e.terrain && e.dynamiccomponent && !(e.dynamiccomponent && (e.dynamiccomponent.name == "Icon") || e.dynamiccomponent.name == "Screen");
}

function IsObjectFocusable(e)
{
    return e.placeable && !e.terrain && e.dynamiccomponent && e.graphicsviewcanvas;
    //return e.placeable && !e.terrain && e.dynamiccomponent && !(e.dynamiccomponent && (e.dynamiccomponent.name == "Icon"));
}
