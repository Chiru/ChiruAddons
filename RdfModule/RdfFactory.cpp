// For conditions of distribution and use, see copyright notice in LICENSE

#include "RdfFactory.h"

#include "RdfModel.h"
#include "RdfNode.h"
#include "RdfStatement.h"
#include "RdfWorld.h"

#include "CoreDefines.h"
#include "Framework.h"
#include "QScriptEngineHelpers.h"

IModel* RdfFactory::CreateModel(IWorld* world)
{
    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    return new RdfXmlModel(rdfWorld);
}

IWorld* RdfFactory::CreateWorld()
{
    return new RdfWorld();
}
