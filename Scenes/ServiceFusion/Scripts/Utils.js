// Utils.js - Misc. utility functions
// Prerequisities:
// engine.ImportExtension("qt.core");
// engine.IncludeFile("MathUtils.js");

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

// QByteArray's toString not exposed in QtScript, must do it manually.
QByteArray.prototype.toString = function()
{
    return new QTextStream(this, QIODevice.ReadOnly).readAll();
}

// Checks whether or not we're allowed to move an object in the scene.
function IsObjectMovable(e)
{
    return e.placeable && !e.terrain && e.dynamiccomponent && !(e.dynamiccomponent && e.dynamiccomponent.name == "Icon");
}