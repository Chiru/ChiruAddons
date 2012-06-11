// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "IScript.h"
#include "ScriptManager.h"

/// Script interface to be used in QtScript.
/** To use this object in QtScript, simply allocate new object and connect to its Invoked signal. */
class Script : public CieMap::IScript
{
    Q_OBJECT

public:
    /// public CieMap::IScript override.
    /** Emits the Invoked signal. */
    void Run(const CieMap::Tag &tag, IMemoryStore *rdfStore)
    {
        emit Invoked(tag, rdfStore);
    }

signals:
    /// Signalled when this script has been invoked.
    /** @param tag Tag that is associated with this script
        @param rdfStore RDF data that is passed to the script. */
    void Invoked(const CieMap::Tag &tag, IMemoryStore *rdfStore);
};