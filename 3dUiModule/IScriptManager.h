// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "3dUiModuleFwd.h"

#include <QObject>

#include <vector>

namespace CieMap
{
/// Interface for the script manager components.
/** Script manager stores registered scripts and they can be queried and run based on tags. */
class IScriptManager : public QObject
{
    Q_OBJECT

public:
    enum
    {
        InvalidId = -1 ///< Invalid script ID.
    };

    /// Registers script based on the tag.
    /** After the script has been registered, it can be retrieved using the returned id.
        Only one script can be registered to one tag and if previously registered script already exists, it is replaced.
        Throws ArgumentException if either parameter is null or empty.
        @param name tag the tag associated with the script
        @param name script the script to register
        @return script id */
    virtual int RegisterScript(const Tag &tag, IScript *script) = 0;

    /// Returns script id based on a tag.
    /** @param tag The tag associated with the script
        @returns Script ID, or InvalidId if no script was registered for the tag */
    virtual std::vector<int> ScriptIdsForTag(const Tag &tag) const = 0;

    /// Runs a script with the specified id.
    /** Throws ArgumentOutOfRangeException if no script with the specified id is found.
        @param id id of the script to run.
        @param tag Tag that is passed to the script.
        @param rdfStore RDF data that is passed to the script. */
    void RunScript(int id, const Tag &tag, RdfMemoryStore *rdfStore);
};

}
