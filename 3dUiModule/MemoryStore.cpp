#include "MemoryStore.h"

#include "IModel.h"
#include "RdfModule.h"
#include "3dUiModule.h"
#include "IWorld.h"
#include "CoreTypes.h"
#include "LoggingFunctions.h"

MemoryStore::MemoryStore(IWorld* world):
    model(0)
{
    assert(world && "World object was a null.");
    if (world)
        model = world->CreateModel();
    else
        LogError("MemoryStore(): Null rdf world object was passed.");
}

MemoryStore::~MemoryStore()
{
    if (model)
        delete model;
}