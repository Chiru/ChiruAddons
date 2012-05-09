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

    void CallScript(const Tag &tag, RdfMemoryStore *refStore);

private:
    ScriptManager *scriptManager;
};

}
