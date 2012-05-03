// For conditions of distribution and use, see copyright notice in LICENSE

#include "3dUiModule.h"
// CieMap types
#include "Container.h"
#include "EventManager.h"
#include "HttpRequestResponse.h"
#include "IContainer.h"
#include "IEventManager.h"
#include "IHttpRequestService.h"
#include "IScript.h"
#include "IScriptManager.h"
#include "IVisualContainer.h"
#include "Layout.h"
#include "Position3.h"
#include "RdfVocabulary.h"
#include "ScriptManager.h"
#include "ScriptServices.h"
#include "MemoryStore.h"
#include "LoggingFunctions.h"

#include "CoreDefines.h"
#include "Framework.h"
#include "QScriptEngineHelpers.h"
#include "SceneAPI.h"
#include "IComponentFactory.h"
#include "DragDropWidget.h"

#include <QScriptEngine>

Q_DECLARE_METATYPE(CieMap::IContainer *)
Q_DECLARE_METATYPE(CieMap::IEventManager *)
Q_DECLARE_METATYPE(CieMap::IVisualContainer *)
Q_DECLARE_METATYPE(CieMap::Layout *)
Q_DECLARE_METATYPE(CieMap::ContainerFactory *)
Q_DECLARE_METATYPE(CieMap::Container *)
Q_DECLARE_METATYPE(CieMap::ScriptManager *)
Q_DECLARE_METATYPE(CieMap::EventManager *)
Q_DECLARE_METATYPE(CieMap::HttpRequestResponse *)
Q_DECLARE_METATYPE(CieMap::Tag *)
Q_DECLARE_METATYPE(CieMap::IScript *)
Q_DECLARE_METATYPE(CieMap::IHttpRequestService *)
Q_DECLARE_METATYPE(CieMap::HttpRequest *)
Q_DECLARE_METATYPE(SemWeb::MemoryStore *)
Q_DECLARE_METATYPE(DragDropWidget *)

CieMapModule::CieMapModule() :
    IModule("CieMap"),
    factory(0)
{
}

CieMapModule::~CieMapModule()
{
}

void CieMapModule::Load()
{
}

void CieMapModule::Initialize()
{
    framework_->RegisterDynamicObject("CieMapModule", this);
//    framework_->Console()->RegisterCommand("test", "test", this, SLOT(Test(const QString &)));
}

QScriptValue CreateDragDropWidget(QScriptContext *ctx, QScriptEngine *engine)
{
    DragDropWidget *w = 0;
    if (ctx->argumentCount() == 0)
        w = new DragDropWidget();
    else if (ctx->argumentCount() == 1)
        w = new DragDropWidget(dynamic_cast<QWidget *>(ctx->argument(0).toQObject()));
    else
        return ctx->throwError(QScriptContext::TypeError, "DragDropWidget(): invalid number of arguments provided.");

    return engine->toScriptValue(w);
}

void CieMapModule::OnScriptEngineCreated(QScriptEngine* engine)
{
    qScriptRegisterQObjectMetaType<CieMap::IContainer *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::IEventManager *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::IVisualContainer *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::Layout *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::ContainerFactory *>(engine);
    /// @todo Doesn't compile
//    qScriptRegisterQObjectMetaType<CieMap::Container *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::ScriptManager *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::EventManager *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::HttpRequestResponse *>(engine);
//    qScriptRegisterQObjectMetaType<CieMap::Tag *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::IScript *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::IHttpRequestService *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::HttpRequest *>(engine);
    /// @todo CieMap::Position3
    qScriptRegisterQObjectMetaType<SemWeb::MemoryStore *>(engine);

    qScriptRegisterQObjectMetaType<DragDropWidget *>(engine);
    QScriptValue ctorDragDropWidget = engine->newFunction(CreateDragDropWidget);
    engine->globalObject().setProperty("DragDropWidget", ctorDragDropWidget);
}

extern "C"
{
DLLEXPORT void TundraPluginMain(Framework *fw)
{
    Framework::SetInstance(fw); // Inside this DLL, remember the pointer to the global framework object.
    fw->RegisterModule(new CieMapModule());
}
}
