
var defaultTransform = me.placeable.transform;

me.Action("Reset").Triggered.connect(function()
{
    me.placeable.transform = defaultTransform;
});

/*
engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

me.Action("MousePress").Triggered.connect(function() {
    QDesktopServices.openUrl(new QUrl("http://grooveshark.com/"));
});
*/
