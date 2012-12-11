engine.IncludeFile("sfdata.js");

SetDropHandler(GotDrop);

function GotDrop(entity) {
    SetInfoQuery("CIE_Updates")
}

SetInfoHandler(GotNews);

function GotNews(json_info) {
    info = JSON.parse(json_info);
    print("setting title to " + info.title);
    widgets.titleLabel.text = info.title;
    var text = ""
    for (var i = 0; i< info.entries.length; i++) {
	print("> " + info.entries[i]);
	text += info.entries[i] + "\n";
    }
    widgets.bodylabel.text = text;
}
