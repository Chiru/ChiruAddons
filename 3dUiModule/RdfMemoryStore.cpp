// For conditions of distribution and use, see copyright notice in LICENSE

#include "RdfMemoryStore.h"

#include "IModel.h"
#include "RdfModule.h"
#include "3dUiModule.h"
#include "IWorld.h"
#include "CoreTypes.h"
#include "LoggingFunctions.h"
#include "CoreDefines.h"

RdfMemoryStore::RdfMemoryStore(IWorld* world):
    model(0)
{
    assert(world && "World object was a null.");
    if (world)
        model = world->CreateModel();
    else
        LogError("RdfMemoryStore(): Null rdf world object was passed.");
}

RdfMemoryStore::~RdfMemoryStore()
{
    SAFE_DELETE(model);
}
