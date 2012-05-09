// For conditions of distribution and use, see copyright notice in LICENSE

//using SemWeb; TODO

#include "ScriptManager.h"
#include "IScript.h"

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
        /// @todo print error throw new System.ArgumentNullException("Parameter cannot be null", "script");
        return InvalidId;
    }
    if (tag.IsEmpty())
    {
        /// @todo print error throw new System.ArgumentException("Parameter cannot be null or empty", "tag");
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

    QMapIterator<Tag, int> it(tags);
    while(it.hasNext())
    {
        it.next();
        if ((it.key() == tag) || (it.key().Type() == tag.Type() && (it.key().Data() == tag.Data() || tag.Data().isEmpty())))
            ids.push_back(it.value());
    }
    return ids;
}

void ScriptManager::RunScript(int id, const Tag &tag, IMemoryStore *rdfStore)
{
    if (id != InvalidId && id >= 0 && id < (int)scripts.size())
    {
        scripts[id]->Run(tag, rdfStore);
    }
    else
    {
        /// @todo print error
        //throw new System.ArgumentOutOfRangeException("Parameter out of range", "id");
    }
}

}
