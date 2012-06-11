engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.network");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");

ui.MainWindow().mouseTracking = true;  

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
    Container.call(this, parent);
    
    this.movies = new Array();
    this.visual.size = new QSize(285, 400);
    me.graphicsviewcanvas.width = this.visual.width;
    me.graphicsviewcanvas.height = this.visual.height;
    this.visual.setLayout(new QVBoxLayout());
    this.visual.setContentsMargins(0, 0, 0, 0);
    this.visual.layout().setSpacing(0);
    
    var world    = RdfModule.theWorld;
    var request  = new HttpRequest();
    var response = ScriptServices.SendPreprocessorRequest("http://hq.ludocraft.com/ludowww/cie/movies2.php",
                                                          "http://www.finnkino.fi/xml/Schedule/?area=1018",
                                                          request); 
    
    response.Ready.connect(this, function(response)
    {
        if (this.visual.owner.rdfStore.FromString(response.data))
        {
            // Create query statement.
            var subject = world.CreateResource(new QUrl("http://cie/news#"));
            var predicate = world.CreateResource(new QUrl(Container.RDFVocabulary.data));
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
                if (movie && movie.time > currentTime.getTime())
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
MovieContainer.prototype = new Container();
 
MovieContainer.prototype.DisplayMovie = function(movie) 
{
    var minStr = movie.time.getMinutes();
    if (minStr == 0)
        minStr = "00";
    var timeStr = movie.time.getHours() + ":" + minStr;

    /*var movieVisual = new VisualContainer(this.visual);
    var container = C3DUiModule.ContainerFactory().CreateContainer(movieVisual);
    container.parent = this.visual.owner;
    container.rdfStore = RdfModule.theWorld.CreateStore();
    movieVisual.setLayout(new QHBoxLayout());
    this.visual.layout().addWidget(movieVisual, null, null);*/
    
    var main = new QWidget();
    main.setLayout(new QHBoxLayout());
    main.setSizePolicy (QSizePolicy.Expanding, QSizePolicy.Preffered);
    main.setContentsMargins(0, 0, 0, 0); 
    main.objectName = "Movie";
    var movieVisual = CreateVisualContainer(main, new QHBoxLayout(), this.visual);
    
    // Fill tag infomation to given MovieContainer.
    var world = RdfModule.theWorld;
    var subject = world.CreateResource(new QUrl("http://cie/"));
    var source = world.CreateResource(new QUrl("http://cie/source-application"));
    var data = world.CreateResource(new QUrl("http://cie/data"));
    var movieSource = world.CreateLiteral("Movie"); 
    var title = world.CreateLiteral(movie.title);
    var auditorium = world.CreateLiteral(movie.auditorium);
    var time = world.CreateLiteral(timeStr);
            
    var statement = world.CreateStatement(subject, source, movieSource);
    movieVisual.owner.rdfStore.AddStatement(statement);
    world.FreeStatement(statement);
    
    statement = world.CreateStatement(subject, data, title);
    movieVisual.owner.rdfStore.AddStatement(statement);
    world.FreeStatement(statement);
    
    statement = world.CreateStatement(subject, data, auditorium);
    movieVisual.owner.rdfStore.AddStatement(statement);
    world.FreeStatement(statement);
    
    statement = world.CreateStatement(subject, data, time);
    movieVisual.owner.rdfStore.AddStatement(statement);
    world.FreeStatement(statement); 
    
    world.FreeNode(subject);
    world.FreeNode(source);
    world.FreeNode(data);
    world.FreeNode(movieSource);
    world.FreeNode(title);
    world.FreeNode(auditorium); 
    world.FreeNode(time);
    
    var label = new QLabel(timeStr); 
    label.font = new QFont("FreeSans", 14);
    label.alignment = 0x0001 | 0x0020; // Qt::AlignLeft | Qt::AlignTop
    main.layout().addWidget(label, null, null);
    
    var label2 = new QLabel(movie.title + "\n" + movie.auditorium);
    label2.font = new QFont("FreeSans", 12);
    label2.setSizePolicy (QSizePolicy.Expanding, QSizePolicy.Preffered);
    main.layout().addWidget(label2, null, null);
    
    var line = CreateHLine();
    this.visual.layout().addWidget(line, null, null);
    
    main.setAttribute(Qt.WA_DeleteOnClose);
    movieVisual.layout().addWidget(main, null, null);
    movieVisual.layout().spacing = 0;
    movieVisual.setContentsMargins(0, 0, 0, 0);
}

function CreateContainer(entity)
{
    entity.graphicsviewcanvas.GraphicsView().mouseTracking = true;
    var container = new MovieContainer(null)
    entity.graphicsviewcanvas.GraphicsScene().addWidget(container.visual);
    return container;
}

var container = CreateContainer(me);

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
}
