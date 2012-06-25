engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");

dragObjectName = "movable_visa_card";

var visaContainer = new BaseContainer(null);
MakeDraggable(visaContainer.visual, null);
AddStatement(visaContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.sourceApplication, "CreditCard");
AddStatement(visaContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "****4321"); // username for bank access
AddStatement(visaContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "****"); // password for bank access
AddStatement(visaContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "823"); // secure number

sceneinteract.EntityClicked.connect(EntityClicked)

function EntityClicked(entity)
{
    if (entity.id == me.id)
    {
        visaContainer.visual.StartDrag(new QPoint(0,0));
    }
}

if (!framework.IsHeadless()) 
{
    //ui.GraphicsView().DropEvent.connect(OnDropEvent);
}

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (visaContainer && visaContainer.visual)
        visaContainer.visual.deleteLater();
}

