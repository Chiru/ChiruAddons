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
#include "ScriptManager.h"
#include "ScriptServices.h"
#include "LoggingFunctions.h"
#include "VisualContainer.h"

#include "CoreDefines.h"
#include "Framework.h"
#include "QScriptEngineHelpers.h"
#include "SceneAPI.h"
#include "IComponentFactory.h"
#include "DragDropWidget.h" 
#include "HttpRequestService.h"

#include "TestScript.h"

#include "RdfModule.h"
#include "IWorld.h"

#include "qscript_tag.h"
#include "qscript_scriptservices.h"

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
//Q_DECLARE_METATYPE(CieMap::Tag *)
Q_DECLARE_METATYPE(CieMap::IScript *)
Q_DECLARE_METATYPE(CieMap::IHttpRequestService *)
Q_DECLARE_METATYPE(CieMap::HttpRequest *)
Q_DECLARE_METATYPE(DragDropWidget *)
Q_DECLARE_METATYPE(CieMap::VisualContainer *)
Q_DECLARE_METATYPE(CieMap::HttpRequestService *)


Q_DECLARE_METATYPE(TestScript *)

C3DUiModule::C3DUiModule() :
    IModule("C3DUi")
{
}

C3DUiModule::~C3DUiModule()
{
    if (containerFactory) delete containerFactory;
}

void C3DUiModule::Load()
{
}

void C3DUiModule::Initialize()
{
    framework_->RegisterDynamicObject("C3DUiModule", this);
    containerFactory = new CieMap::ContainerFactory();
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

QScriptValue CreateVisualContainer(QScriptContext *ctx, QScriptEngine *engine)
{
    CieMap::VisualContainer *c = 0;
    if (ctx->argumentCount() == 1)
    {
        c = new CieMap::VisualContainer(dynamic_cast<QWidget *>(ctx->argument(1).toQObject()));
    }
    else
        return ctx->throwError(QScriptContext::TypeError, "VisualContainer(): invalid number of arguments provided.");

    return engine->toScriptValue(c);
}

QScriptValue CreateTestScript(QScriptContext *ctx, QScriptEngine *engine)
{
    CieMap::IScript *s = 0;
    if (ctx->argumentCount() == 0)
    {
        s = new TestScript();
    }
    //else
    //    return ctx->throwError(QScriptContext::TypeError, "VisualContainer(): invalid number of arguments provided.");

    return engine->toScriptValue(s);
}

QScriptValue CreateHttpRequest(QScriptContext *ctx, QScriptEngine *engine)
{
    CieMap::HttpRequestService *h = 0;
    if (ctx->argumentCount() == 0)
    {
        h = new CieMap::HttpRequestService();
    }
    //else
    //    return ctx->throwError(QScriptContext::TypeError, "VisualContainer(): invalid number of arguments provided.");

    return engine->toScriptValue(h);
}

void C3DUiModule::OnScriptEngineCreated(QScriptEngine* engine)
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
    //qScriptRegisterQObjectMetaType<CieMap::Tag *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::IScript *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::IHttpRequestService *>(engine);
    qScriptRegisterQObjectMetaType<CieMap::HttpRequest *>(engine);
    /// @todo CieMap::Position3

    qScriptRegisterQObjectMetaType<DragDropWidget *>(engine);
    QScriptValue ctorDragDropWidget = engine->newFunction(CreateDragDropWidget);
    engine->globalObject().setProperty("DragDropWidget", ctorDragDropWidget);

    qScriptRegisterQObjectMetaType<CieMap::VisualContainer *>(engine);
    QScriptValue ctorVisualContainer = engine->newFunction(CreateVisualContainer);
    engine->globalObject().setProperty("VisualContainer", ctorVisualContainer);

    qScriptRegisterQObjectMetaType<TestScript *>(engine);
    QScriptValue ctorTestScript = engine->newFunction(CreateTestScript);
    engine->globalObject().setProperty("TestScript", ctorTestScript);

    qScriptRegisterQObjectMetaType<CieMap::HttpRequestService *>(engine);
    QScriptValue ctorHttpRequest = engine->newFunction(CreateHttpRequest);
    engine->globalObject().setProperty("HttpRequest", ctorHttpRequest);

    register_tag_prototype(engine);
    register_scriptservice_prototype(engine);

    /*qScriptRegisterQObjectMetaType<CieMap::Tag *>(engine);
    QScriptValue ctorTag = engine->newFunction(CreateTag);
    engine->globalObject().setProperty("Tag", ctorTag);*/
}

CieMap::ContainerFactory* C3DUiModule::ContainerFactory() const
{
    return containerFactory;
}

extern "C"
{
DLLEXPORT void TundraPluginMain(Framework *fw)
{
    Framework::SetInstance(fw); // Inside this DLL, remember the pointer to the global framework object.
    fw->RegisterModule(new C3DUiModule());
}
}
