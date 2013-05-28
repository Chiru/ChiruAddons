engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("VisualContainerUtils.js");
engine.IncludeFile("WebViewInterface.js");

if (!framework.IsHeadless())
{
    //ui.GraphicsView().DragMoveEvent.connect(OnDropEvent);
    ui.GraphicsView().DropEvent.connect(OnDropEvent);
}

function OnDropEvent(e)
{
    var data = e.mimeData().data("application/x-hotspot").toString();
    if (data.length > 0)
    {
        var ray = CurrentMouseRay();
        var r = scene.ogre.Raycast(ray, -1);

        if (r.entity && r.entity.id == me.id)
        {
            var searchWord = findChild(e.source(), "search");
            if(searchWord)
            {
                print("youtube: " +searchWord.property("searchWord"));
                var youtubeURL = "http://www.youtube.com/results?search_query=" +searchWord.property("searchWord")+ "&oq" +
                                 searchWord.property("searchWord") +"&gs_l=youtube.3..0l10.10860.12578.0.12738.9.6.0.3.3.0.67.323.6.6.0...0.0...1ac.1.vQeKMfl7Zqc";
                //print("google url: " + googleURL);
                setURL(youtubeURL);
            }
        }
    }
}
