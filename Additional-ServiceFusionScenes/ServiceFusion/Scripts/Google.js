engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("VisualContainerUtils.js");
engine.IncludeFile("WebViewInterface.js");
engine.IncludeFile("SceneContent.js");

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
                var googleURL;
                if(searchWord.property("searchType") == "user")
                {
                    for(var key in SAVED_URLS)
                    {
                        if(key===searchWord.property("searchWord"))
                        {
                            googleURL = SAVED_URLS[key]; 
                            break;
                        }
                    }
                }
                else
                    googleURL = "http://www.google.com/search?q=" +(searchWord.property("searchWord")+ "&ie=UTF-8&sa=Search&channel=fe");//&client=browser-ubuntu&hl=en");
               
               //print("google url: " + googleURL);
               setURL(googleURL);
            }
        }
    }
}
