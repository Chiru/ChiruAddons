//!ref: Tweet.ui
engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("VisualContainerUtils.js");
engine.IncludeFile("RdfVocabulary.js");

var tweets = [];

var show_debug = false;

function debug(arg) {
    if (show_debug)
        print(arg)
}

debug("Twitter.js loading");

me.dynamiccomponent.CreateAttribute("string", "searchWord");
me.dynamiccomponent.CreateAttribute("string", "searchType");
me.dynamiccomponent.CreateAttribute("string", "result");

me.dynamiccomponent.AttributeChanged.connect(ShowSearcResults);
var twitterScreen = scene.GetEntityByName("twitter_screen");

if (!framework.IsHeadless())
{
    ui.GraphicsView().DropEvent.connect(OnDropEvent);
    //ui.GraphicsView().DragMoveEvent.connect(OnMoveEvent);
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
            //var searchWord = findChild(e.source(), "song_artist")
            var searchWord = findChild(e.source(), "search");
            if(searchWord)
            {
                //print("twitterSearch: " +searchWord.property("searchWord") + " " +searchWord.property("searchType"));
                DoSearch(searchWord.property("searchWord"), searchWord.property("searchType"));
                frame.DelayedExecute(0.2).Triggered.connect( function() { sceneinteract.EntityClicked(me, 0x00000001, r); });
            }
            else
                print("no luck");
        }
    }
}


function DoSearch(searchWord, searchType)
{
    //searchWordParsed = ((String)(searchWord.split("\n", 1))).replace(/\s/g, '');
    searchWordParsed = searchWord.replace(/\s/g, '');
    print("twitter: searching for " + searchWordParsed);
    me.dynamiccomponent.SetAttribute("searchWord", searchWordParsed);
    me.dynamiccomponent.SetAttribute("searchType", searchType);
}


function ShowSearcResults(attribute, attrchange)
{

    titleWidget = new QWidget();
    titleWidget.setLayout(new QVBoxLayout());
    titleLabel = new QLabel("Tweets");
    titleLabel.font = new QFont("SansSerif", 42);
    titleLabel.alignment = 0x0004; // Qt::AlignHCenter
    //openLabel = new QLabel("----------");
    //openLabel.font = new QFont("SansSerif", 16);
    //openLabel.alignment = 0x0004;
    titleWidget.layout().addWidget(titleLabel, 0, 0);
    //titleWidget.layout().addWidget(openLabel, 0, 0);
    var title = twitterScreen.GetComponent("EC_GraphicsViewCanvas", "Title");
    titleWidget.size = new QPoint(title.width, title.height);
    title.GraphicsScene().addWidget(titleWidget);
    title.GraphicsView().styleSheet = "background-color:white;";
    titleWidget.styleSheet = "background-color:white;";
    titleWidget.show();

    if(attribute.name=="result")
    {
        //var container = CreateContainer(twitterScreen);
        tweets = attribute.value.split("tweetContent:");
        twitterWidget = new QWidget();
        var scrollArea = new QScrollArea(twitterWidget);
        var layout = new QVBoxLayout(twitterWidget);
        var scrollAreaLayout = new QVBoxLayout();
        var scrollAreaContent = new QWidget(scrollAreaLayout);
        for(i=0; i<tweets.length; i++)
        {
            if(tweets[i] != "")
            {
                var data = {};
                var tweetCell = asset.GetAsset("Tweet.ui").Instantiate(false, 0);
                tweetCell.visible = true;
                data.user = tweets[i].slice(tweets[i].indexOf("'user':")+8, tweets[i].indexOf("'user_image_url':")-2);
                data.text = tweets[i].slice(tweets[i].indexOf("'text':")+8, tweets[i].indexOf("'retweets':")-2);
                data.imageUrl = tweets[i].slice(tweets[i].indexOf("'user_image_url':")+18, tweets[i].indexOf("'location':")-2);
                findChild(tweetCell, "UserLabel").setText(data.user);
                findChild(tweetCell, "TextLabel").setText(data.text);
                RequestImage(data.imageUrl, "Texture", tweetCell);
                scrollAreaLayout.addWidget(tweetCell,0,0);             
            }   
        }

        var gvc = twitterScreen.GetComponent("EC_GraphicsViewCanvas", "Content");
        //scrollAreaContent.styleSheet = "background-color:#00ACED;";
        scrollAreaContent.setLayout(scrollAreaLayout);
        scrollArea.setWidget(scrollAreaContent);
        layout.addWidget(scrollArea,0,0);
        twitterWidget.resize(gvc.width, gvc.height);
        gvc.GraphicsScene().addWidget(twitterWidget);
        gvc.GraphicsView().styleSheet = "background-color:white;";
        twitterWidget.styleSheet = "background-color:white;";//"background-color:#00ACED;";
        twitterWidget.show();

        me.dynamiccomponent.SetAttribute("searchWord", "");
        me.dynamiccomponent.SetAttribute("searchType", "");

    }
}

function RequestImage(ref, type, widget)
{
    //asset.ForgetAsset(ref, true);

    var transferPtr = asset.RequestAsset(ref, type, true);
    transferPtr.Succeeded.connect(function(a) {

        var imageLabel = findChild(widget, "ImageLabel");
        if(a.IsLoaded())
            imageLabel.setPixmap(new QPixmap(a.DiskSource()));
    });

    transferPtr.Failed.connect(function(a, r) {});
}
