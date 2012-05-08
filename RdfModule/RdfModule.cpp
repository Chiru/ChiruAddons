// For conditions of distribution and use, see copyright notice in LICENSE

#include "RdfModule.h"

#include "RdfModel.h"
#include "RdfNode.h"
#include "RdfStatement.h"
#include "RdfWorld.h"

#include "CoreDefines.h"
#include "Framework.h"
#include "QScriptEngineHelpers.h"

#include <QScriptEngine>

Q_DECLARE_METATYPE(RdfXmlModel *)
Q_DECLARE_METATYPE(RdfNode *)
Q_DECLARE_METATYPE(RdfWorld *)
Q_DECLARE_METATYPE(RdfStatement *)

Q_DECLARE_METATYPE(IModel *)
Q_DECLARE_METATYPE(INode *)
Q_DECLARE_METATYPE(IWorld *)
Q_DECLARE_METATYPE(IStatement *)

RdfModule::RdfModule() :
    IModule("Rdf")
{
}

RdfModule::~RdfModule()
{
}

void RdfModule::Load()
{
}

void RdfModule::Initialize()
{
    framework_->RegisterDynamicObject("RdfModule", this);
    world = new RdfWorld();
//    framework_->Console()->RegisterCommand("test", "test", this, SLOT(Test(const QString &)));
}

void RdfModule::OnScriptEngineCreated(QScriptEngine* engine)
{
    // RDF objects.
    qScriptRegisterQObjectMetaType<RdfXmlModel *>(engine);
    qScriptRegisterQObjectMetaType<RdfNode *>(engine);
    qScriptRegisterQObjectMetaType<RdfWorld *>(engine);
    qScriptRegisterQObjectMetaType<RdfStatement *>(engine);

    qScriptRegisterQObjectMetaType<IModel *>(engine);
    qScriptRegisterQObjectMetaType<INode *>(engine);
    qScriptRegisterQObjectMetaType<IWorld *>(engine);
    qScriptRegisterQObjectMetaType<IStatement *>(engine);

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
