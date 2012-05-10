engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

me.Action("MousePress").Triggered.connect(function() {
    QDesktopServices.openUrl(new QUrl("http://grooveshark.com/"));
});
