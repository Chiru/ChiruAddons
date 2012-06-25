engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");

dragObjectName = "movable_card";

var idContainer = new BaseContainer(null);
MakeDraggable(idContainer.visual, null);
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.sourceApplication, "ID");
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "John");
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "Doe");
var birthday = new Date("October 13, 1975");
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, birthday.toString());
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "Male");
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "johndoe@mymail.com");
AddStatement(idContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "358 04 1234567");

sceneinteract.EntityClicked.connect(EntityClicked)

function EntityClicked(entity)
{
	if (entity.id == me.id)
	{
		idContainer.visual.StartDrag(new QPoint(0,0));
	}
}

var cardEntity = scene.GetEntityByName("movable_card");

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

