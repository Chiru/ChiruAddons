// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "IScript.h"
#include "ScriptManager.h"

class TestScript : public CieMap::IScript
{
    Q_OBJECT
    public:
    /// Run the script
    /** @param tag Tag that is associated with this script
        @param rdfStore RDF data that is passed to the script. */
    void Run(const CieMap::Tag &tag, IMemoryStore *rdfStore);
};