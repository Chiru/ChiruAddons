engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.network");
engine.IncludeFile("RdfVocabulary.js");

ui.MainWindow().mouseTracking = true;

QByteArray.prototype.toString = function()
{
    ts = new QTextStream( this, QIODevice.ReadOnly );
    return ts.readAll();
}

function Movie()
{
    this.title = "";
    this.auditorium = "";
    this.time = null;
}

Movie.prototype.IsEmpty = function()
{
    if (this.title == "" && this.auditorium == "" && this.time == null)
        return true;
    return false;
}

function Container(parent)
{
    this.visual = new VisualContainer(parent);
    this.container = C3DUiModule.ContainerFactory().CreateContainer(this.visual, parent);
    this.container.rdfStore = new RdfMemoryStore(RdfModule.theWorld);
}

function MovieContainer(parent)
{
    Container.call(this, parent);
    
    this.movies = new Array();
    this.visual.size = new QSize(370, 200);
    this.visual.setLayout(new QVBoxLayout());
    
    //\todo Use HttpResponse object instead --Joosua.
    var world      = RdfModule.theWorld;
    this.store     = world.CreateStore();
    this.subject   = world.CreateResource(new QUrl("http://cie/news#"));
    this.predicate = world.CreateResource(new QUrl("http://cie/data-source"));
    this.object    = world.CreateLiteral("http://www.finnkino.fi/xml/Schedule/?area=1018");
    this.statement = world.CreateStatement(this.subject, this.predicate, this.object);
    this.store.AddStatement(this.statement);
    
    var url = new QUrl("http://hq.ludocraft.com/ludowww/cie/movies2.php");
    var request = new QNetworkRequest(url);
    request.setRawHeader("User-Agent", "realXtend Tundra");
    var accessManager = new QNetworkAccessManager();
    
    accessManager.finished.connect(this, function(reply)
    {
        var byteArray = reply.readAll();
        var store = this.visual.owner.rdfStore;
        if (store.FromString(byteArray))
        {
            // Query return RDF data as list of statements.
            var statements = store.Select(world.CreateStatement(world.CreateResource(new QUrl("http://cie/news#")),
                                          world.CreateResource(new QUrl(RdfVocabulary.data)),
                                          null));
            // Parse each statment to movie object.
            var movie = null;
            for(var i = 0; i < statements.length; ++i)
            {
                movie = this.ParseTextToMovie(statements[i].object.literal.toString());
                if (movie)
                    this.movies.push(movie);
            }

            // Display three movie objects in GraphicsViewCanvas.
            // \todo add date time check what movies are about to start and display them.
            if (this.movies.length > 3)
            {
                this.DisplayMovie(this.movies[0]);
                this.DisplayMovie(this.movies[1]);
                this.DisplayMovie(this.movies[2]);
            }
        }
    });

    var post_data = this.store.toString();
    accessManager.post(request, new QByteArray("data=" + post_data));
}
MovieContainer.prototype = new Container();

MovieContainer.prototype.ParseTextToMovie = function(data) 
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
}

MovieContainer.prototype.DisplayMovie = function(movie) 
{
    var movieVisual = new VisualContainer(this.visual);
    var movieContainer = C3DUiModule.ContainerFactory().CreateContainer(movieVisual, this.visual);
    movieContainer.rdfStore = new RdfMemoryStore(RdfModule.theWorld);
    var label = new QLabel(movie.title + "\n" + movie.auditorium + "\n" + movie.time.getHours() + ":" + movie.time.getMinutes(), movieVisual);
    label.font = new QFont("Arial", 9);
    label.setAttribute(Qt.WA_DeleteOnClose);
    movieVisual.size = label.sizeHint;
    this.visual.layout().addWidget(movieVisual, 0, 0);
}

function CreateContainer(entity)
{
    entity.graphicsviewcanvas.GraphicsView().mouseTracking = true;
    var container = new MovieContainer(null)
    entity.graphicsviewcanvas.GraphicsScene().addWidget(container.visual);
}

CreateContainer(me);

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    // \todo MovieContainer not released.
}