engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.network");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");

ui.MainWindow().mouseTracking = true;  

var titleWidget = null;

function Movie()
{
    this.title = "";
    this.auditorium = "";
    this.time = null;
}
 
Movie.IsEmpty = function()
{
    if (this.title == "" && this.auditorium == "" && this.time == null) 
        return true; 
    return false;
};

// Construct a movie struct from the string, if parse fails null object is returned.
Movie.FromString = function(data)
{
    var movie = null;
    var v = data.split(";");  
    if (v.length >= 3)
    {
        movie = new Movie();
        // Note! fromString should return QDateTime object, but javascript's Date object is returned instead.  
        var date = QDateTime.fromString(v[2], "yyyy-MM-ddThh:mm:ss");
        movie.title = v[0];
        movie.auditorium = v[1]; 
        movie.time = date;
    }
    return movie;
};

function MovieContainer(parent)
{
    BaseContainer.call(this, parent);
    this.visual.styleSheet = "background-color:white;";

    titleWidget = new QWidget();
    titleWidget.setLayout(new QVBoxLayout());
    titleLabel = new QLabel("FINNKINO");
    titleLabel.font = new QFont("SansSerif", 42);
    titleLabel.alignment = 0x0004; // Qt::AlignHCenter
    openLabel = new QLabel("\nAVOINNA 11-23"); // Could use spacer here, but going with newline for now
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
    
    this.movies = new Array();
    this.visual.size = new QSize(320, 400);
    me.graphicsviewcanvas.width = this.visual.width;
    me.graphicsviewcanvas.height = this.visual.height;
    this.visual.setLayout(new QVBoxLayout());
    this.visual.layout().setContentsMargins(0, 0, 0, 0);
    this.visual.layout().setSpacing(0);
    
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
            
            // Parse each statment to movie object.
            var movie = null;
            var currentTime = new Date();
            for(var i = 0; i < statements.length; ++i)
            {
                movie = Movie.FromString(statements[i].object.literal.toString());
                if (/*movie && movie.time > currentTime.getTime() &&*/ movie.auditorium == "sali 1" || movie.auditorium == "sali 2")
                    this.movies.push(movie);
                world.FreeStatement(statements[i]);
            }
            
            var displayCount = 5;
            for (var i = 0; i < this.movies.length; ++i)
            {
                if (i < displayCount)
                    this.DisplayMovie(this.movies[i]);
                else
                    break;
            }
        }
        response.deleteLater(); 
        response = null;
    });
}
MovieContainer.prototype = new BaseContainer();

// Create VisualContainer object for single movie and add it as child to MovieContainer object.
MovieContainer.prototype.DisplayMovie = function(movie) 
{
    var main = new QWidget();
    main.styleSheet = "background-color:white;";
    main.setLayout(new QHBoxLayout());
    main.setSizePolicy (QSizePolicy.Expanding, QSizePolicy.Preffered);
    main.layout().setContentsMargins(5, 0, 0, 0); 
    main.objectName = "Movie";
    
    var movieVisual = CreateVisualContainer(main, new QHBoxLayout(), this.visual);
    
    // Fill tag infomation to given movie container.
    // Note! AddStatement function is defined in VisualContainerUtils.js file.
    AddStatement(movieVisual, RdfVocabulary.baseUri, RdfVocabulary.sourceApplication, "Movie");
    AddStatement(movieVisual, RdfVocabulary.baseUri, RdfVocabulary.data, movie.time.toString());
    AddStatement(movieVisual, RdfVocabulary.baseUri, RdfVocabulary.data, movie.title);
    AddStatement(movieVisual, RdfVocabulary.baseUri, RdfVocabulary.data, movie.auditorium);
    
    // Initialize time Label 
    var minStr = movie.time.getMinutes();
    if (minStr == 0)
        minStr = "00";
    var timeStr = movie.time.getHours() + ":" + minStr;
    var label = new QLabel(timeStr); 
    label.font = new QFont("FreeSans", 12);
    label.alignment = 0x0001 | 0x0020; // Qt::AlignLeft | Qt::AlignTop
    main.layout().addWidget(label, null, null);
    
    // Initialize title/auditorium Label 
    var label2 = new QLabel(movie.title + "\n" + movie.auditorium.toUpperCase());
    label2.font = new QFont("FreeSans", 12);
    label2.setSizePolicy (QSizePolicy.Expanding, QSizePolicy.Preffered);
    label2.styleSheet = "margin-left:20px;";
    main.layout().addWidget(label2, null, null);
    
    var line = CreateHLine();
    this.visual.layout().addWidget(line, 0, 0);
    
    movieVisual.layout().addWidget(main, null, null); 
    movieVisual.layout().spacing = 0;
    movieVisual.setContentsMargins(0, 0, 0, 0);
}

function CreateContainer(entity)
{
    var container = new MovieContainer(null)
    entity.graphicsviewcanvas.GraphicsScene().addWidget(container.visual);
    //print(entity.graphicsviewcanvas.GraphicsView().childrenRegion.boundingRect());
    return container;
}

var container = CreateContainer(me);
// Register ourselves to the Movie icon
scene.EntityByName("IconScript").Exec(1, "RegisterInfoBubble", "movie_icon_h", me.id.toString());

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

