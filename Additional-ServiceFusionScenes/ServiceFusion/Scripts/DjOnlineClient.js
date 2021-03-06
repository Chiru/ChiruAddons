engine.ImportExtension("qt.core");
engine.ImportExtension("qt.xml");
engine.ImportExtension("qt.network");

function get_child_element(doc, eltname) {
    var x = doc.elementsByTagName(eltname);
    var elt = null;
    if (x.length() > 0)
elt = x.item(0).toElement();
    if (!elt)
throw "Element " + eltname + " not found";
    return elt;
}

function parse_description(descr) {
    //descr = descr.replace(/^Now playing: /, '');
    if (descr.match(/^Now playing: /))
return null;
    descr = descr.replace(/^Next: /, '');
    if (!descr.match(/^\d\d:/))
descr = "00:00:00 " + descr;
    print("incoming " + descr);

    //descr = descr.replace(/^\d+: /, '');
    //descr = descr.replace(/^Now playing: /, '');
    //descr = descr.replace(/^Next: /, '');
    var re = /(\d\d:\d\d:\d\d|Now playing: |)\s+([^-]+) - ([^\)\(]*)/;
    print("XXX " + descr);
    var matchinfo = re.exec(descr);
    if (!matchinfo) {
print("parse_description: bad description '" + descr + "' so returning null");

return null;
    }
    return {"playtime": matchinfo[1].trim(),
"artist": matchinfo[2],
"song": matchinfo[3].trim(),
}
}


function parse_rss(rss_bytearray) {
    var domdoc = new QDomDocument("RSS");
    /*var file = new QFile("t.rss");
file.open(QIODevice.ReadOnly);
var rc = domdoc.setContent(file); */
    var rc = domdoc.setContent(rss_bytearray);
    if (!rc)
throw("dom document parse failed");
    var chan_elt = get_child_element(domdoc, "channel");
    var items = chan_elt.elementsByTagName("item");
    var out = new Array();

    var always_song = {
playtime: "00:00:00",
artist: "Darin",
song: "See U At The Club"
    }
    out.push(always_song);
    
    for (var i = 0; i < items.length(); i++) {
var item_elt = items.item(i).toElement();
var title_string = get_child_element(item_elt, "title").text();
var description_string = get_child_element(item_elt, "description").text();
// if (i)
// print("--");
// print(title_string);
parsed = parse_description(description_string);
if (!parsed)
continue;
if (parsed.artist != "Darin")
out.push(parsed);
print("pushing: " + parsed.artist);
// print(parsed.playtime);
// print(parsed.artist);
// print(parsed.song);
    }

    return out;
}

var qnam = new QNetworkAccessManager();

function start_http_request(url, callback)
{
    var url = new QUrl(url);
    var request = new QNetworkRequest(url);
    print("starting http req " + url);
    qnam.finished.connect(callback);
    // qnam.error.connect(function(errcode) {
    // print("djonline http req failed: " + request.errorString());
    // });
    qnam.get(request);
}


function djonline_get_playlist(site_name, parsed_callback) {
    if (site_name == '1bar')
var url = "http://www.djonline.fi/playing/?id=118";
    else if (site_name == 'ottok')
var url = "http://www.djonline.fi/playing/?id=487";
    else if (site_name == '117')
var url = "http://www.djonline.fi/playing/?id=117";
    else if (site_name == '116')
var url = "http://www.djonline.fi/playing/?id=116";
    else if (site_name == '116')
var url = "http://www.djonline.fi/playing/?id=118";

    function req_finished_callback(reply) {
print("req finish callback");
var data = parse_rss(reply.readAll());
parsed_callback(data);
print("called parsed_callback");
reply.deleteLater();
    }

    start_http_request(url, req_finished_callback);
}
