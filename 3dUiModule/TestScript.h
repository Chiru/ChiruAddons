// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "IScript.h"
#include "ScriptManager.h"

/// Example Service Fusion test script implemented in C++.
class TestScript : public CieMap::IScript
{
    Q_OBJECT

public:
    /// CieMap::IScript override.
    void Run(const CieMap::Tag &tag, IMemoryStore *rdfStore);
};
