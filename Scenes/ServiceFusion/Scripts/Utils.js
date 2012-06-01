// Utils.js - Misc. utility functions
// Prerequisities:
// engine.ImportExtension("qt.core");
// engine.IncludeFile("MathUtils.js");

// TODO: description why this function is needed
function IsTouchStateMoved(touch)
{
    return touch.state() == Qt.TouchPointMoved && !EqualsQPointF(touch.pos(), touch.lastPos());
}

// TODO: description why this function is needed
function IsTouchStateStationary(touch)
{
    return touch.state() == Qt.TouchPointStationary || EqualsQPointF(touch.pos(), touch.lastPos());

}

// QByteArray's toString not exposed in QtScript, must do it manually.
QByteArray.prototype.toString = function()
{
    return new QTextStream(this, QIODevice.ReadOnly).readAll();
}

function IsObjectMovable(e)
{
    return e.placeable && !e.terrain && e.dynamiccomponent && !(e.dynamiccomponent && e.dynamiccomponent.name == "Icon");
}
