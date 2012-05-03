// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "IModule.h"
#include "RdfModuleApi.h"

class QScriptEngine;
class RdfFactory;

class RDF_MODULE_API RdfModule: public IModule
{
    Q_OBJECT

public:
    RdfModule();
    virtual ~RdfModule();

    void Load();
    void Initialize();

public slots:
    RdfFactory* GetRdfFactory();

private slots:
    void OnScriptEngineCreated(QScriptEngine* engine);

private:
    RdfFactory* factory;
};
