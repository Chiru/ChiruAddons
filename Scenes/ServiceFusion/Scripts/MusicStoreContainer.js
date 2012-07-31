engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.network");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");
engine.IncludeFile("Log.js");
engine.IncludeFile("Localisation.js");

var titleWidget = null;

function Song()
{
    this.title = "";
    this.artist = "";
    this.price = 0;
}

Song.IsEmpty = function()
{
    if (this.title == "" && this.artist == "" && this.price == 0)
        return true;
    return false;
};

// Construct a movie struct from the string, if parse fails null object is returned.
Song.FromString = function(data)
{
    var song = null;
    var params = data.split("\n");
    if(params.length == 2)
    {
        song = new Song();
        song.title = params[0];
        song.artist = params[1];
    }
    return song;
};

LOC_MUSIC_TITLE = "Music store";
LOC_MUSIC_OPEN = "Open 9-21";

function MusicStoreContainer(parent)
{
    BaseContainer.call(this, parent);
    this.visual.styleSheet = "background-color:white;";

    titleWidget = new QWidget();
    titleWidget.setLayout(new QVBoxLayout());
    titleLabel = new QLabel(LOC_MUSIC_TITLE);
    titleLabel.font = new QFont("SansSerif", 42);
    titleLabel.alignment = 0x0004; // Qt::AlignHCenter
    openLabel = new QLabel("\n" + LOC_MUSIC_OPEN); // Could use spacer here, but going with newline for now
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

    this.songs = new Array();
    this.visual.size = new QSize(320, 400);
    me.graphicsviewcanvas.width = this.visual.width;
    me.graphicsviewcanvas.height = this.visual.height;
    this.visual.setLayout(new QVBoxLayout());
    this.visual.layout().setContentsMargins(0, 0, 0, 0);
    this.visual.layout().setSpacing(0);

    // Since we don't have rdf source for any real online music store, fake search for now.
    ui.GraphicsView().DropEvent.connect(this, this.OnDropEvent);

//    var testSong = new Song();
//    testSong.artist = "Carly Comando";
//    testSong.title = "Everyday";
//    testSong.price = 2;
//    this.DisplaySong(testSong);

/*
    var world = RdfModule.theWorld;
    var request = new HttpRequest();
    request.operation = QNetworkAccessManager.GetOperation;
    var response = ScriptServices.SendPreprocessorRequest(
            "http://hq.ludocraft.com/ludowww/cie/movies2.php",
            "http://www.finnkino.fi/xml/Schedule/?area=1018",
            request);

    response.Ready.connect(this, function(response)
                           {
                               if (this.visual.owner.rdfStore.FromString(response.data))
                               {
                                   // Create query statement.
                                   var subject = world.CreateResource(new QUrl("http://cie/news#"));
                                   var predicate = world.CreateResource(new QUrl(RdfVocabulary.data));
                                   var statement = world.CreateStatement(subject, predicate, null);

                                   var statements = this.visual.owner.rdfStore.Select(statement);

                                   world.FreeNode(subject);
                                   world.FreeNode(predicate);
                                   world.FreeStatement(statement);

                                   var song = null;
                                   for(var i = 0; i < statements.length; ++i)
                                   {
                                       // Parse songs here
                                       this.songs.push(song);
                                       world.FreeStatement(statements[i]);
                                   }

                                   var displayCount = 5;
                                   for (var i = 0; i < this.movies.length; ++i)
                                   {
                                       if (i < displayCount)
                                           this.DisplaySong(this.songs[i]);
                                       else
                                           break;
                                   }
                               }
                               response.deleteLater();
                               response = null;
                           });
*/
}
MusicStoreContainer.prototype = new BaseContainer();

// Create VisualContainer object for single movie and add it as child to MovieContainer object.
MusicStoreContainer.prototype.DisplaySong = function(song)
{
    var main = new QWidget();
    main.styleSheet = "background-color:white;";
    main.setLayout(new QHBoxLayout());
    main.setSizePolicy (QSizePolicy.Expanding, QSizePolicy.Preffered);
    main.layout().setContentsMargins(5, 0, 0, 0);
    main.objectName = "Song";

    var musicVisual = CreateVisualContainer(main, new QHBoxLayout(), this.visual);

    // Fill tag infomation to given movie container.
    // Note! AddStatement function is defined in VisualContainerUtils.js file.
    AddStatement(musicVisual, RdfVocabulary.baseUri, RdfVocabulary.sourceApplication, "MusicStore");
    AddStatement(musicVisual, RdfVocabulary.baseUri, RdfVocabulary.data, song.price.toString());
    AddStatement(musicVisual, RdfVocabulary.baseUri, RdfVocabulary.data, song.title);
    AddStatement(musicVisual, RdfVocabulary.baseUri, RdfVocabulary.data, song.artist);

    // Initialize title/artist Label
    var label2 = new QLabel(song.title + "\n" + song.artist.toUpperCase());
    label2.font = new QFont("FreeSans", 12);
    label2.setSizePolicy (QSizePolicy.Expanding, QSizePolicy.Preffered);
    label2.styleSheet = "margin-right:20px;";
    main.layout().addWidget(label2, null, null);

    // Initalize price label
    var label = new QLabel(song.price.toString() + "e");
    label.font = new QFont("FreeSans", 18, 75);
    label.alignment = 0x0002 | 0x0020; // Qt::AlignLeft | Qt::AlignTop
    main.layout().addWidget(label, null, null);

    var line = CreateHLine();
    this.visual.layout().addWidget(line, 0, 0);

    musicVisual.layout().addWidget(main, null, null);
    musicVisual.layout().spacing = 0;
    musicVisual.setContentsMargins(0, 0, 0, 0);
}

MusicStoreContainer.prototype.OnDropEvent = function(e)
{
    var data = e.mimeData().data("application/x-hotspot").toString();
    if (data.length > 0)
    {
        var songLabel = findChild(e.source(), "song_artist")
        if(!songLabel)
            return;

        var iconEntity = scene.GetEntityByName("MusicStoreIcon");
        var entityVisible = iconEntity.dynamiccomponent.GetAttribute("infoBubbleVisible");

        var ray = CurrentMouseRay();
        var r = scene.ogre.Raycast(ray, -1);

        // Only react to drops when the song label isn't dropped on an entity that has a dynamiccomponent
        if((r.entity && !r.entity.dynamiccomponent) || !r.entity)
        {
            // Trigger sceneinteract's EntityClicked -signal which is then intercepted by IconsCript.js
            if(entityVisible === undefined || entityVisible === false)
                sceneinteract.EntityClicked(iconEntity, 0x00000001, r);

            var song = Song.FromString(songLabel.text);
            song.price = Math.floor(Math.random() * 5 + 1); // Random price since this is fake search
            if(song)
                this.DisplaySong(song);
        }
    }
}

function CreateContainer(entity)
{
    var container = new MusicStoreContainer(null)
    entity.graphicsviewcanvas.GraphicsScene().addWidget(container.visual);
    //print(entity.graphicsviewcanvas.GraphicsView().childrenRegion.boundingRect());
    return container;
}

var container = CreateContainer(me);

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (titleWidget)
    {
        titleWidget.deleteLater();
        titleWidget = 0;
    }
    if (container)
    {
        container.visual.deleteLater();
        container = null;
    }
}
