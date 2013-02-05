engine.IncludeFile("sfdata.js");

SetInfoHandler(GotNews);

function GotNews(json_info) {
    print("GotNews called");
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
