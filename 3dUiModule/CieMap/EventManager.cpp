// For conditions of distribution and use, see copyright notice in LICENSE

#include "EventManager.h"
#include "ScriptManager.h"
#include "CoreDefines.h"

namespace CieMap
{

EventManager::EventManager() : scriptManager(new ScriptManager())
{
}

EventManager::~EventManager()
{
    SAFE_DELETE(scriptManager);
}

int EventManager::RegisterScript(const Tag &tag, IScript *script)
{
    return scriptManager->RegisterScript(tag, script);
}

bool EventManager::HasScript(const Tag &tag) const
{
    return (scriptManager->ScriptIdsForTag(tag).size() > 0);
}

void EventManager::CallScript(const Tag &tag, IMemoryStore *rdfStore)
{
    foreach(int id, scriptManager->ScriptIdsForTag(tag))
        scriptManager->RunScript(id, tag, rdfStore);
}

}
