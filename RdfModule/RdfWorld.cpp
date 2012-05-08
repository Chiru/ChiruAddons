// For conditions of distribution and use, see copyright notice in LICENSE

#include "RdfWorld.h"
#include "RdfModel.h"
#include "RdfNode.h"
#include "RdfStatement.h"
#include "LoggingFunctions.h"

RdfWorld::RdfWorld()
{
    world = librdf_new_world();
    librdf_world_open(world);
    storage = librdf_new_storage(world, "memory", NULL, NULL);
    parser = librdf_new_parser(world, "rdfxml", NULL, NULL);
    serializer = librdf_new_serializer(world, "rdfxml", NULL, NULL);
}

RdfWorld::~RdfWorld()
{
    if (serializer)
        librdf_free_serializer(serializer);
    if (parser)
        librdf_free_parser(parser);
    if (storage)
        librdf_free_storage(storage);
    if (world)
        librdf_free_world(world);
}

IModel* RdfWorld::CreateModel()
{
    return new RdfXmlModel(this);
}

INode* RdfWorld::CreateResource(QUrl uri)
{
    return new RdfNode(uri, this);
}

INode* RdfWorld::CreateNode()
{
    return new RdfNode(this);
}
INode* RdfWorld::CreateLiteral(QString lit_v)
{
    return new RdfNode(lit_v, this);
}

IStatement* RdfWorld::CreateStatement(IStatement* statement)
{
    return statement->Clone();
}

IStatement* RdfWorld::CreateStatement(INode* subject, INode* predicate, INode* object)
{
    return new RdfStatement(this, subject, predicate, object);;
}