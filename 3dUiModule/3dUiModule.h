// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "IModule.h"

class QScriptEngine;
class IWorld;

namespace CieMap
{
    class ContainerFactory;
}

class C3DUiModule: public IModule
{
    Q_OBJECT

public:
    C3DUiModule();
    virtual ~C3DUiModule();

    void Load();
    void Initialize();

public slots:
    CieMap::ContainerFactory* ContainerFactory() const;

private slots:
    void OnScriptEngineCreated(QScriptEngine* engine);

private:
    CieMap::ContainerFactory* containerFactory;
};
