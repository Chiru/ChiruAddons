// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "IModule.h"

class QScriptEngine;
namespace CieMap
{
    class CieFactory;
}

class CieMapModule: public IModule
{
    Q_OBJECT

public:
    CieMapModule();
    virtual ~CieMapModule();

    void Load();
    void Initialize();

private slots:
    void OnScriptEngineCreated(QScriptEngine* engine);

private:
    CieMap::CieFactory* factory;
};
