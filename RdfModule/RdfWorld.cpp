// For conditions of distribution and use, see copyright notice in LICENSE

#include "RdfWorld.h"
#include "RdfMemoryStore.h"
#include "RdfNode.h"
#include "RdfStatement.h"
#include "LoggingFunctions.h"

RdfWorld::RdfWorld():
    parser(0),
    serializer(0)
{
    world = librdf_new_world();
    librdf_world_open(world);
    parser = librdf_new_parser(world, "rdfxml", NULL, NULL);
    serializer = librdf_new_serializer(world, "rdfxml", NULL, NULL);
}

RdfWorld::~RdfWorld()
{
    if (serializer)
        librdf_free_serializer(serializer);
    if (parser)
        librdf_free_parser(parser);
    if (world)
        librdf_free_world(world);
}

IMemoryStore* RdfWorld::CreateStore() 
{
    RdfMemoryStore* store = new RdfMemoryStore(this);
    return store;
}

void RdfWorld::FreeStore(IMemoryStore *store)
{
    if (store) delete store;
}

INode* RdfWorld::CreateResource(QUrl uri)
{
    RdfNode* node = new RdfNode(uri, this);
    return node;
}

INode* RdfWorld::CreateNode()
{
    RdfNode* node = new RdfNode(this);
    return node;
}
INode* RdfWorld::CreateLiteral(QString lit_v)
{
    RdfNode* node = new RdfNode(lit_v, this);
    return node;
}

void RdfWorld::FreeNode(INode *node)
{
    if (node) delete node;
}

IStatement* RdfWorld::CreateStatement(INode* subject, INode* predicate, INode* object)
{
    RdfStatement* s = new RdfStatement(this, subject, predicate, object);
    return s;
}

void RdfWorld::FreeStatement(IStatement *statement)
{
    delete statement;
}

void RdfWorld::HandleNodeDestroy(QObject *obj)
{
    INode *n = static_cast<INode *>(obj);
    if (n) FreeNode(n);
}

void RdfWorld::HandleStatementDestroy(QObject *obj)
{
    RdfStatement *s = static_cast<RdfStatement *>(obj);
    if (s) FreeStatement(s);
}

void RdfWorld::HandleStoreDestroy(QObject *obj)
{
    RdfMemoryStore *m = static_cast<RdfMemoryStore *>(obj);
    if (m) FreeStore(m);
}