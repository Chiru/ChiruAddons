#pragma once

#include "3dUiModuleFwd.h"

#include <QObject>

namespace CieMap
{
/// Handles events originating from active region in containers.
class IEventManager : public QObject
{
    Q_OBJECT

public:
    IEventManager() {}
    virtual ~IEventManager() {}

    /// Registers script based on the tag.
    /** After the script has been registered, it can be retrieved using the returned id.
        Only one script can be registered to one tag and if previously registered script already exists, it is replaced.

        Throws ArgumentException if either parameter is null or empty.
        @param tag the tag associated with the script
        @param script the script to register
        @return script id */
    virtual int RegisterScript(const Tag &tag, IScript *script) = 0;

    /// Returns true if there is a script associated with the specified tag
    /** @param name tag Tag to test */
    virtual bool HasScript(const Tag &tag) const = 0;

    /// Calls a script based on the specified tag
    /** @param tag Tag associated with the script
        @param rdfStore RDF data that is passed to the script. */
    virtual void CallScript(const Tag &tag, SemWeb::MemoryStore *rdfStore) = 0;
};

}
