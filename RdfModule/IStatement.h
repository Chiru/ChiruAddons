// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include <QObject>

#include "CoreTypes.h"

class INode;
class IWorld;
class IModel;

class IStatement : public QObject
{
    Q_OBJECT

    friend class IModel;
    friend class IWorld;

public:
    IStatement(IWorld* world) : world(world),
        subject(0),
        predicate(0),
        object(0)
    {
    }

    IStatement(IWorld* world, INode *subject, INode *predicate, INode *object) : world(world),
        subject(subject),
        predicate(predicate),
        object(object)
    {
    }

    virtual ~IStatement()
    {
    }

    virtual IStatement* Clone() = 0;

    /// Checks if the statement is complete and valid.
    virtual bool IsValid() = 0;

    INode* Subject() const
    {
        return subject;
    }

    INode* Predicate() const
    {
        return predicate;
    }

    INode* Object() const
    {
        return object;
    }

    IWorld* World() const
    {
        return world;
    }

protected:
    /// World object that have created this statement.
    IWorld *world;
    /// Node type resource
    INode *subject;
    /// Node type resource
    INode *predicate;
    /// Node type literal
    INode *object;
};

//Q_DECLARE_INTERFACE(IStatement, "Tundra.RdfModule.IStatement")