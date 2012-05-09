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

    int RegisterScript(const Tag &tag, IScript *script);

    bool HasScript(const Tag &tag) const;

    void CallScript(const Tag &tag, IMemoryStore *refStore);

private:
    ScriptManager *scriptManager;
};

}