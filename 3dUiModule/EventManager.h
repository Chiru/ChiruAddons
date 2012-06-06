// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "IEventManager.h"
#include "3dUiModuleFwd.h"

namespace CieMap
{
/// Implementation of IEventManager interface.
class EventManager : public IEventManager
{
    Q_OBJECT

public:
    EventManager();
    ~EventManager();

public slots:
    int RegisterScript(const CieMap::Tag &tag, CieMap::IScript *script); 

    bool HasScript(const CieMap::Tag &tag) const;

    void CallScript(const CieMap::Tag &tag, IMemoryStore *refStore);

private:
    ScriptManager *scriptManager;
};

}
