// For conditions of distribution and use, see copyright notice in LICENSE

#include "RdfModule.h"

#include "RdfMemoryStore.h"
#include "RdfNode.h"
#include "RdfStatement.h"
#include "RdfWorld.h"

#include "CoreDefines.h"
#include "Framework.h"
#include "QScriptEngineHelpers.h"

#include <QScriptEngine>
//#include <vld.h> 

IWorld *RdfModule::worldInstance = 0;

Q_DECLARE_METATYPE(RdfMemoryStore *)
Q_DECLARE_METATYPE(RdfNode *)
Q_DECLARE_METATYPE(RdfWorld *)
Q_DECLARE_METATYPE(RdfStatement *)

Q_DECLARE_METATYPE(IMemoryStore *)
Q_DECLARE_METATYPE(INode *)
Q_DECLARE_METATYPE(IWorld *)
Q_DECLARE_METATYPE(IStatement *)

RdfModule::RdfModule() :
    IModule("Rdf")
{
}

RdfModule::~RdfModule()
{
    if (world) delete world;
}

void RdfModule::Load()
{
}

void RdfModule::Initialize()
{
    framework_->RegisterDynamicObject("RdfModule", this);
    world = new RdfWorld();
    worldInstance = world;
}

QScriptValue CreateRdfMemoryStore(QScriptContext *ctx, QScriptEngine *engine)
{
    RdfMemoryStore *s = 0;
    if (ctx->argumentCount() == 1)
    {
        QObject* obj = ctx->argument(0).toQObject();
        IWorld* world = dynamic_cast<IWorld *>(obj);
        s = new RdfMemoryStore(world);
    }
    else
        return ctx->throwError(QScriptContext::TypeError, "RdfMemoryStore(): invalid number of arguments provided.");

    return engine->toScriptValue(s);
}

void RdfModule::OnScriptEngineCreated(QScriptEngine* engine)
{
    // RDF objects.
    qScriptRegisterQObjectMetaType<RdfMemoryStore *>(engine);
    qScriptRegisterQObjectMetaType<RdfNode *>(engine);
    qScriptRegisterQObjectMetaType<RdfWorld *>(engine);
    qScriptRegisterQObjectMetaType<RdfStatement *>(engine);

    qScriptRegisterQObjectMetaType<IMemoryStore *>(engine);
    qScriptRegisterQObjectMetaType<INode *>(engine);
    qScriptRegisterQObjectMetaType<IWorld *>(engine);
    qScriptRegisterQObjectMetaType<IStatement *>(engine);

    qScriptRegisterQObjectMetaType<RdfMemoryStore *>(engine);
    QScriptValue ctorRdfMemoryStore = engine->newFunction(CreateRdfMemoryStore);
    engine->globalObject().setProperty("RdfMemoryStore", ctorRdfMemoryStore);

    qRegisterMetaType<RdfNode::NodeType>("NodeType");
}

IWorld* RdfModule::GetWorld() const
{
    return world;
}

extern "C"
{
DLLEXPORT void TundraPluginMain(Framework *fw)
{
    Framework::SetInstance(fw); // Inside this DLL, remember the pointer to the global framework object.
    fw->RegisterModule(new RdfModule());
}
}
