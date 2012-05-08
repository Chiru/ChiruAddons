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
#include "VisualContainer.h"

#include "CoreDefines.h"
#include "Framework.h"
#include "QScriptEngineHelpers.h"
#include "SceneAPI.h"
#include "IComponentFactory.h"
#include "DragDropWidget.h"

#include "RdfModule.h"
#include "IWorld.h"

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
Q_DECLARE_METATYPE(CieMap::VisualContainer *)

CieMapModule::CieMapModule() :
    IModule("CieMap")
{
}

CieMapModule::~CieMapModule()
{
    if (containerFactory) delete containerFactory;
}

void CieMapModule::Load()
{
}

void CieMapModule::Initialize()
{
    framework_->RegisterDynamicObject("CieMapModule", this);
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

QScriptValue CreateMemoryStore(QScriptContext *ctx, QScriptEngine *engine)
{
    SemWeb::MemoryStore *s = 0;
    if (ctx->argumentCount() == 1)
    {
        QObject* obj = ctx->argument(0).toQObject();
        IWorld* world = dynamic_cast<IWorld *>(obj);
        s = new SemWeb::MemoryStore(world);
    }
    else
        return ctx->throwError(QScriptContext::TypeError, "MemoryStore(): invalid number of arguments provided.");

    return engine->toScriptValue(s);
}

/*QScriptValue CreateContainer(QScriptContext *ctx, QScriptEngine *engine)
{
    QScriptValue module = engine->globalObject().property("CieMapModule");
    if (module.isNull()) 
        return ctx->throwError(QScriptContext::UnknownError, "CreateVisualContainer(): Couldn't find an instance of \"CieMapModule\" object.");

    CieMap::VisualContainer *c = 0;
    if (ctx->argumentCount() == 1)
    {
        c = new CieMap::VisualContainer(dynamic_cast<QWidget *>(ctx->argument(1).toQObject()));
        QList<CieMap::IVisualContainer*> args;
        args.push_back(c);
        QScriptValue* retVal = module.property("ContainerFactory").property("CreateContainer").call(engine->globalObject(), new QScriptValueList(&args));
        module.setProperty("tempContainer", retVal);
    }
    else
        return ctx->throwError(QScriptContext::TypeError, "VisualContainer(): invalid number of arguments provided.");

    return engine->toScriptValue(c);
}*/

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
    QScriptValue ctorMemoryStore = engine->newFunction(CreateMemoryStore);
    engine->globalObject().setProperty("MemoryStore", ctorMemoryStore);

    qScriptRegisterQObjectMetaType<DragDropWidget *>(engine);
    QScriptValue ctorDragDropWidget = engine->newFunction(CreateDragDropWidget);
    engine->globalObject().setProperty("DragDropWidget", ctorDragDropWidget);

    qScriptRegisterQObjectMetaType<CieMap::VisualContainer *>(engine);
    QScriptValue ctorVisualContainer = engine->newFunction(CreateVisualContainer);
    engine->globalObject().setProperty("VisualContainer", ctorVisualContainer);


    /*QScriptValue ctorVisualContainer = engine->newFunction(CreateContainer);
    engine->globalObject().setProperty("Container", ctorContainer);*/
}

CieMap::ContainerFactory* CieMapModule::ContainerFactory() const
{
    return containerFactory;
}

extern "C"
{
DLLEXPORT void TundraPluginMain(Framework *fw)
{
    Framework::SetInstance(fw); // Inside this DLL, remember the pointer to the global framework object.
    fw->RegisterModule(new CieMapModule());
}
}
