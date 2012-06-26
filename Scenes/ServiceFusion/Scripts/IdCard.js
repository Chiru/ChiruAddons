engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");

dragObjectName = "movable_id_card";

var idContainer = new BaseContainer(null);
MakeDraggable(idContainer.visual, null);
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.sourceApplication, "ID");
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "Daniel");
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "Tanner");
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "dtanner@mymail.com");
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "0401234567");
var birthday = new Date("October 13, 1975");
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, birthday.toString());
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "Male");

sceneinteract.EntityClicked.connect(EntityClicked)

function EntityClicked(entity)
{
    if (entity.id == me.id)
        idContainer.visual.StartDrag(new QPoint(0,0));
}

if (!framework.IsHeadless())
{
    //ui.GraphicsView().DropEvent.connect(OnDropEvent);
}

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
	if (idContainer && idContainer.visual)
		idContainer.visual.deleteLater();
}

