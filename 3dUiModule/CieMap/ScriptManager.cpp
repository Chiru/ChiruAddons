// For conditions of distribution and use, see copyright notice in LICENSE

#include "ScriptManager.h"
#include "IScript.h"
#include "LoggingFunctions.h"

namespace CieMap
{

int ComputeSdbmHash(const QString &str)
{
    int ret = 0;
    if (!str.length())
        return ret;
    const char* cstr = str.toStdString().c_str();
    while(*cstr)
    {
        // Note: calculate case-insensitive hash
        char c = *cstr;
        ret = tolower(c) + (ret << 6) + (ret << 16) - ret;
        ++cstr;
    }
    return ret;
}

int ScriptManager::RegisterScript(const Tag &tag, IScript *script)
{
    if (!script)
    {
        LogError("ScriptManager::RegisterScript: Script parameter cannot be null");
        return InvalidId;
    }
    if (tag.IsEmpty())
    {
        LogError("ScriptManager::RegisterScript: Tag parameter cannot be null");
        return InvalidId;
    }

    int id = scripts.size();
    scripts.push_back(script);
    tags[tag] = id;
    return id;
}

std::vector<int> ScriptManager::ScriptIdsForTag(const Tag &tag) const
{
    std::vector<int> ids;

    if (tag.IsEmpty())
    {
        LogError("ScriptIdsForTag(const Tag&): Empty tag as parameter.");
        return ids;
    }

    QMapIterator<Tag, int> it(tags);
    while(it.hasNext())
    {
        it.next();
        if (!it.key().IsEmpty() && (it.key().Type() == tag.Type() || it.key().Data() == tag.Data()))
            ids.push_back(it.value());
    }
    return ids;
}

void ScriptManager::RunScript(int id, const Tag &tag, IMemoryStore *rdfStore)
{
    if (id != InvalidId && id >= 0 && id < (int)scripts.size())
        scripts[id]->Run(tag, rdfStore);
    else
        LogError(QString("ScriptManager::RunScript: out-of-range ID %1 given.").arg(id));
}

}
