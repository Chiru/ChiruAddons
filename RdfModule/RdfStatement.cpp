// For conditions of distribution and use, see copyright notice in LICENSE

#include "RdfStatement.h"
#include "RdfNode.h"
#include "RdfWorld.h"
#include "RdfMemoryStore.h"
#include "LoggingFunctions.h"

RdfStatement::RdfStatement(IWorld* world) : IStatement(world)
{
    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    assert(rdfWorld && "Failed to dynamic cast IWorld to RdfWorld");

    if (rdfWorld)
    {
        rdfWorld->RegisterStatement(this);
        statement = librdf_new_statement(rdfWorld->world);
    }
    else
        LogError("Failed to cast IWorld to RdfWorld.");
}

RdfStatement::RdfStatement(IWorld* world, INode *subject, INode *predicate, INode *object) : IStatement(world, subject, predicate, object)
{
    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    assert(rdfWorld && "Failed to dynamic cast IWorld to RdfWorld"); 

    if (rdfWorld)
    {
        rdfWorld->RegisterStatement(this);
        RdfNode *rdfSubject   = dynamic_cast<RdfNode*>(subject);
        RdfNode *rdfPredicate = dynamic_cast<RdfNode*>(predicate);
        RdfNode *rdfObject    = dynamic_cast<RdfNode*>(object);

        if (rdfSubject)
            this->subject = new RdfNode(rdfSubject, world);
        if (rdfPredicate)
            this->predicate = new RdfNode(rdfPredicate, world);
        if (rdfObject)
            this->object = new RdfNode(rdfObject, world);

        librdf_node* s = rdfSubject ? rdfSubject->node : 0;
        librdf_node* p = rdfPredicate ? rdfPredicate->node : 0;
        librdf_node* o = rdfObject ? rdfObject->node : 0;
        statement = librdf_new_statement_from_nodes(rdfWorld->world, s, p, o);
    }
    else
        LogError("Failed to cast IWorld to RdfWorld.");
}

RdfStatement::RdfStatement(IWorld* world, librdf_statement* statement) :
  IStatement(world)
{
    RdfNode *s = new RdfNode(librdf_statement_get_subject(statement), world);
    RdfNode *p = new RdfNode(librdf_statement_get_predicate(statement), world);
    RdfNode *o = new RdfNode(librdf_statement_get_object(statement), world);

    subject = s;
    predicate = p;
    object = o;

    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    assert(rdfWorld && "Failed to dynamic cast IWorld to RdfWorld");

    if (rdfWorld)
    {
        this->statement = librdf_new_statement_from_nodes(rdfWorld->world, s->node, p->node, o->node);
        rdfWorld->RegisterStatement(this);
    }
    else
        LogError("Failed to cast IWorld to RdfWorld.");
}

RdfStatement::~RdfStatement()
{
    if (statement)
        librdf_free_statement(statement);
}

bool RdfStatement::IsValid()
{
    return !librdf_statement_is_complete(statement) ? true : false;
}