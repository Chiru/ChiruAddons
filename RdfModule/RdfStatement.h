// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "RdfModuleFwd.h"
#include "CoreTypes.h"
#include "IStatement.h"

#include <redland.h>
#include <QObject>

class RdfStatement : public IStatement
{
    Q_OBJECT
    Q_PROPERTY(IWorld* world READ World)
    Q_PROPERTY(INode* subject READ Subject)
    Q_PROPERTY(INode* predicate READ Predicate)
    Q_PROPERTY(INode* object READ Object)

public:
    RdfStatement(IWorld* world);
    RdfStatement(IWorld* world, INode *subject, INode *predicate, INode *object);
    RdfStatement(IWorld* world, librdf_statement* statement);
    
    virtual ~RdfStatement();

    virtual RdfStatement* Clone();

    /// Checks if the statement is complete and valid.
    virtual bool IsValid();

    librdf_statement* statement;
};
