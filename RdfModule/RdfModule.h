// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "IModule.h"
#include "RdfModuleApi.h"

class QScriptEngine;
class RdfFactory;
class IWorld;

class RDF_MODULE_API RdfModule: public IModule
{
    Q_OBJECT
    Q_PROPERTY(IWorld* theWorld READ GetWorld)

public:
    RdfModule();
    virtual ~RdfModule();

    void Load();
    void Initialize();
    IWorld* GetWorld() const;

private slots:
    void OnScriptEngineCreated(QScriptEngine* engine);

private:
    /// World object that wraps Redland Rdf world object.
    /// Note! Redland will only allow single isntance of the world object.
    IWorld* world;
};
