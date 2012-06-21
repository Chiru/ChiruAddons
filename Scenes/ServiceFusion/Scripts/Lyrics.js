
/* 

 - feed of played songs from a bar
 - current song should be a selectable object that can be dragged onto 
   radio
 - this triggers a search for songs lyrics
 - lyrics include a selectable object for the artist
*/

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.ImportExtension("qt.network");
engine.IncludeFile("Log.js");

QByteArray.prototype.toString = function()
{
    ts = new QTextStream( this, QIODevice.ReadOnly );
    ts.setCodec("UTF-8");
    return ts.readAll();
}

var qnam = new QNetworkAccessManager();

function MakeLyricsUrl(song)
{
    var titlepart = song.title.replace(/ +/g, '-');
    var artistpart = song.artist.replace(/ +/g, '-');
    return ('http://www.lyrics.com/' + titlepart + '-lyrics-' + artistpart + '.html').toLowerCase();
}

function GetSongLyrics(song, callback) {
    StartRequest(MakeLyricsUrl(song), song, callback);
}

function StartRequest(url, song, callback)
{
    var url = new QUrl(url);
    var request = new QNetworkRequest(url);
    print("lyrics url " + url);
    qnam.finished.connect(function(reply) { HttpFinished(reply, song, callback); });
    qnam.get(request);
}

function HttpFinished(reply, song, callback)
{
    var byteArray = reply.readAll();
    var stringCode = byteArray.toString();
    var strStart = '<div id="lyric_space">';
    var strStart2 = '<div id="lyrics" class="SCREENONLY">'
    var strEnd = '<br />---<br />';
    var start = stringCode.search(strStart);
    var end = stringCode.search(strEnd);
    if (end === -1) {
	print("no lyrics found, end was -1");
	return null;
    }
    var strLyrics = stringCode.substring(start + strStart.length, end);
    strLyrics = strLyrics.replace(/<br \/>/g, " -");

    Log("Calling callback with lyrics");
    callback(song, strLyrics);
}


