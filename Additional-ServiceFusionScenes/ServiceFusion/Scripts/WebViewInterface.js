engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.webkit");

var URL = null;

var webViewEntity = scene.GetEntityByName("news_tablet");
/*function getGvcSize()
{
    var gvc = me.graphicsviewcanvas;
    return (new QSize(gvc.width, gvc.height));
}*/

function setURL(url)
{
    setWebView(url);   
}

function setWebView(urlAddress)
{
    var webView = new QGraphicsWebView();
    webView.url = new QUrl(urlAddress);
    webView.size = new QSize(webViewEntity.graphicsviewcanvas.width, webViewEntity.graphicsviewcanvas.height);
    webViewEntity.graphicsviewcanvas.GraphicsScene().addItem(webView);
}
