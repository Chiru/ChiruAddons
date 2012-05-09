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
        statement = librdf_new_statement(rdfWorld->world);
    else
        LogError("Failed to cast IWorld to RdfWorld.");
}

RdfStatement::RdfStatement(IWorld* world, INode *subject, INode *predicate, INode *object) : IStatement(world, subject, predicate, object)
{
    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    assert(rdfWorld && "Failed to dynamic cast IWorld to RdfWorld");

    if (rdfWorld)
    {
        statement = librdf_new_statement(rdfWorld->world);

        RdfNode *rdfSubject   = dynamic_cast<RdfNode*>(subject);
        RdfNode *rdfPredicate = dynamic_cast<RdfNode*>(predicate);
        RdfNode *rdfObject    = dynamic_cast<RdfNode*>(object);

        if (rdfSubject)
            librdf_statement_set_subject(statement, rdfSubject->node);
        if (rdfPredicate)
            librdf_statement_set_predicate(statement, rdfPredicate->node);
        if (rdfObject)
            librdf_statement_set_object(statement, rdfObject->node);

        //\ todo use shared pointers with nodes.
        this->subject = rdfSubject;
        this->predicate = rdfPredicate;
        this->object = rdfObject;
    }
    else
        LogError("Failed to cast IWorld to RdfWorld.");
}

RdfStatement::RdfStatement(IWorld* world, librdf_statement* statement) :
  IStatement(world)
{
    subject = new RdfNode(librdf_statement_get_subject(statement), world);
    predicate = new RdfNode(librdf_statement_get_predicate(statement), world);
    object = new RdfNode(librdf_statement_get_object(statement), world);
    this->statement = statement;
}

RdfStatement::~RdfStatement()
{
    if (statement)
        librdf_free_statement(statement);
}

RdfStatement* RdfStatement::Clone()
{
    return 0;
}

bool RdfStatement::IsValid()
{
    return !librdf_statement_is_complete(statement) ? true : false;
}