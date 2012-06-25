engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.network");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");
engine.IncludeFile("Lyrics.js");

var lyricslabel = null;

me.Action("ShowLyrics").Triggered.connect(function(type, rdfStoreData) {
    print("starting lyrics fetch");
    var barent = scene.GetEntityByName("BarContainer");
    last_song = barent.dynamiccomponent.GetAttribute("last_song");
    if (!last_song) {
        print("no last_song in bar");
        return;
    }

    print("got entity");
    GetSongLyrics(last_song, function(song, lyrics) {
        print("got lyrics in barcontainer");
        if (!lyricslabel)
            print("no lyricslabel");
        else {
            lyricslabel.text = song.title + "\n\n" + lyrics;
        }
    });
});

function LyricsContainer(parent)
{
    BaseContainer.call(this, parent);
    this.visual.size = new QSize(512, 512);
    var contentCanvas = me.GetComponent("EC_GraphicsViewCanvas", "Content");
    contentCanvas.styleSheet = "background-color:white;";
    contentCanvas.width = this.visual.width;
    contentCanvas.height = this.visual.height;
    this.visual.setLayout(new QVBoxLayout());
    this.visual.layout().setContentsMargins(0, 0, 0, 0);
    this.visual.layout().setSpacing(0);
    this.DisplayLyrics("");
}
LyricsContainer.prototype = new BaseContainer();

// Create VisualContainer object for single lyrics and add it as child to LyricsContainer object.

LyricsContainer.prototype.DisplayLyrics = function(lyrics) 
{

    // actually we get the lyrics here since there seems to be no
    // mechanism to pass the parsed lyrics objects out?

    var main = new QWidget();
    main.styleSheet = "background-color:white;";
    main.setLayout(new QVBoxLayout());
//    main.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preffered); -Stinkfist
    main.layout().setContentsMargins(10, 0, 0, 0); 
    main.objectName = "Lyrics";
    
    var lyricsVisual = CreateVisualContainer(main, new QHBoxLayout(), this.visual);
    
    
    // Initialize title Label 
    lyricslabel = new QLabel(lyrics);
    lyricslabel.styleSheet = "background-color:white;";
    lyricslabel.font = new QFont("FreeSans", 12);
    lyricslabel.wordWrap = true;
//    lyricslabel.setSizePolicy (QSizePolicy.Expanding, QSizePolicy.Preffered);  -Stinkfist
    main.layout().addWidget(lyricslabel, null, null);
    
//    var line = CreateHLine();  -Stinkfist
//    lyricsVisual.layout().addWidget(line, 0, 0);  -Stinkfist
    
    lyricsVisual.layout().addWidget(main, null, null);
    lyricsVisual.layout().spacing = 0;
    lyricsVisual.setContentsMargins(0, 0, 0, 0);
}

function CreateContainer(entity)
{
    var container = new LyricsContainer(null)
    entity.graphicsviewcanvas.GraphicsScene().addWidget(container.visual);
    //print(entity.graphicsviewcanvas.GraphicsView().childrenRegion.boundingRect());
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
