// For conditions of distribution and use, see copyright notice in LICENSE

#include "LoggingFunctions.h"
#include "IMemoryStore.h"
#include "IWorld.h"
#include "TestScript.h"

void TestScript::Run(const CieMap::Tag &tag, IMemoryStore *rdfStore)
{
    LogInfo("This is test script.");
}