// !ref: ContainerMovie.ui

engine.ImportExtension("qt.core");
engine.IncludeFile("print.js");
engine.ImportExtension("qt.network");
engine.ImportExtension("qt.gui");

var widget = asset.GetAsset("ContainerMovie.ui").Instantiate(false, null);

QByteArray.prototype.toString = function()
{
    ts = new QTextStream( this, QIODevice.ReadOnly );
    return ts.readAll();
}

function parseDateFromString(dateString)
{
    var variables = dateString.split("T");
}


var world   = RdfFactory.CreateWorld();
var model   = RdfFactory.CreateModel(world);

var subject   = world.CreateResource(new QUrl("http://cie/news#"));
var predicate = world.CreateResource(new QUrl("http://cie/data-source"));
var object    = world.CreateLiteral("http://www.finnkino.fi/xml/Schedule/?area=1018");
var statement = world.CreateStatement(subject, predicate, object);
model.AddStatement(statement);

//var url = new QUrl("http://hq.ludocraft.com/ludowww/cie/movies.php");
var url = new QUrl("http://localhost/movies.php");
var request = new QNetworkRequest(url);
request.setRawHeader("User-Agent", "realXtend Tundra");
var accessManager = new QNetworkAccessManager();

function ParseMovieData(data)
{
    var v = data.split(";"); 
    if (v.length >= 3)
    {
        // Note! fromString should return QDateTime object, but instead javascript Date object is returned.
        var date = QDateTime.fromString(v[2], "yyyy-MM-ddThh:mm:ss");
        var text = v[0] + "\n" + v[1] + "\n" + date.getHours() + ":" + date.getMinutes();
        return text;
    }
    return "";
}

// Triggered when the html response is sent to back.
accessManager.finished.connect(function(reply)
{
    var byteArray = reply.readAll();
    if (model.FromString(byteArray))
    {
        var statements = model.Select(world.CreateStatement(world.CreateResource(new QUrl("http://cie/news#")),
                                                            world.CreateResource(new QUrl("http://cie/data")),
                                                            null));

        if (statements.length > 6)
        {
            var parsedText = "";
            var children = ["movie_label_1", "movie_context_1", "movie_label_2", "movie_context_2", "movie_label_3", "movie_context_3"];
            for(var i = 0; i < 6; ++i)
            {
                parsedText = ParseMovieData(statements[i].object.literal.toString());
                var label = findChild(widget, children[i]);
                if (label) label.text = parsedText;
            }
        }
    }
});

widget.show();

var post_data = model.toString();
accessManager.post(request, new QByteArray("data=" + post_data));