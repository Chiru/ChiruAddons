engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.network");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");
engine.IncludeFile("Lyrics.js");
engine.IncludeFile("DjOnlineClient.js");
ui.MainWindow().mouseTracking = true;
engine.IncludeFile("Log.js");
engine.IncludeFile("Localisation.js");

var show_debug = false;

function debugmsg(msg) {
    if (show_debug)
print(msg);
}

var last_song = null; // updated from Song.FromString


function SongContainer(parent)
{
    BaseContainer.call(this, parent);
    debugmsg("SongContainer init");
    this.songs = new Array();
    this.visual.styleSheet = "background-color:white;";
    this.visual.size = new QSize(320, 400);
    me.graphicsviewcanvas.width = this.visual.width;
    me.graphicsviewcanvas.height = this.visual.height;
    this.visual.setLayout(new QVBoxLayout());
    this.visual.layout().setContentsMargins(0, 0, 0, 0);
    this.visual.layout().setSpacing(0);
    
    titleWidget = new QWidget();
    titleWidget.setLayout(new QVBoxLayout());
    titleLabel = new QLabel("Songs");
    titleLabel.font = new QFont("SansSerif", 42);
    titleLabel.alignment = 0x0004; // Qt::AlignHCenter
    openLabel = new QLabel("----------"); // Could use spacer here, but going with newline for now
    openLabel.font = new QFont("SansSerif", 16);
    openLabel.alignment = 0x0004;
    titleWidget.layout().addWidget(titleLabel, 0, 0);
    titleWidget.layout().addWidget(openLabel, 0, 0);
    var title = me.GetComponent("EC_GraphicsViewCanvas", "Title");
    titleWidget.size = new QPoint(title.width, title.height);
    title.GraphicsScene().addWidget(titleWidget);
    title.GraphicsView().styleSheet = "background-color:white;";
    titleWidget.styleSheet = "background-color:white;";
    titleWidget.show();
    
    var self = this;
    djonline_get_playlist("ottok", function(songs) {
        {
            var displayCount = 5;
debugmsg("got playlist with " + songs.length + " songs");
            for (var i = 0; i < songs.length; ++i)
            {
                if (i < displayCount) {
                    self.DisplaySong(songs[i]);
                } else
                    break;
            }
        }
    });
}
SongContainer.prototype = new BaseContainer();

var label2 = null;
var label = null;
var tw = null;

SongContainer.prototype.DisplaySong = function(song)
{

    // actually we get the lyrics here since there seems to be no
    // mechanism to pass the parsed song objects out?

    debugmsg("displaysong " + song.song);
    var main = new QWidget();
    main.setLayout(new QHBoxLayout());
    main.setSizePolicy (QSizePolicy.Expanding, QSizePolicy.Preffered);
    main.layout().setContentsMargins(10, 0, 0, 0);
    main.objectName = "Song";

    tw = new QWidget(main);
    tw.objectName = "search";
    tw.setProperty("searchWord", song.artist);
    tw.setProperty("searchType", "content");

    /*
    tw.setProperty("as_string", song.artist);
    tw.setProperty("type", "song/title");

    tw.setProperty("as_string", song.artist);
    tw.setProperty("type", "twitter/user");
    tw.setParent(main);
    */
    var songVisual = CreateVisualContainer(main, new QHBoxLayout(), this.visual);
    titleLabel = new QLabel("----------");
    var title = me.GetComponent("EC_GraphicsViewCanvas", "Title");
    titleLabel.size = new QPoint(title.width, title.height);
    title.GraphicsScene().addWidget(titleLabel);
    titleLabel.show();

    label2 = new QLabel("<font color=\"red\">" + song.artist +"</font><br>" + song.song);
    debugmsg("setting label texts");
    label2.objectName = "song_artist";
    label2.alignment = 0x1 | 0x20
    label2.font = new QFont("FreeSans", 24);
    label2.setSizePolicy (QSizePolicy.Expanding, QSizePolicy.Preffered);
    main.layout().addWidget(label2, null, null);
    
    var line = CreateHLine();
    this.visual.layout().addWidget(line, 0, 0);
    
    songVisual.layout().addWidget(main, null, null);
    songVisual.layout().spacing = 0;
    songVisual.setContentsMargins(0, 0, 0, 0);
}

function CreateContainer(entity)
{
    var container = new SongContainer(null)
    entity.graphicsviewcanvas.GraphicsScene().addWidget(container.visual);
    return container;
}

var container = CreateContainer(me);

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (container)
    {
        container.visual.deleteLater();
        container = null;
    }
}
