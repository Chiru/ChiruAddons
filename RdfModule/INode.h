// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include <QUrl>

#include "RdfModuleFwd.h"
#include "CoreTypes.h"
#include "IWorld.h"

class INode : public QObject
{
    Q_OBJECT
    Q_PROPERTY(IWorld* world READ World)
    Q_PROPERTY(QUrl uri READ Uri)
    Q_PROPERTY(QString literal READ Lit)
    Q_PROPERTY(NodeType type READ Type)
    Q_ENUMS(NodeType)

    friend class IStatement;

public:
    enum NodeType
    {
        Empty = 0,
        Blank,
        Literal,
        Resource
    };

    /// Construct a new resource node.
    INode(QUrl uri, IWorld* world):
        world(world),
        uri(uri)
    {
    }

    /// Construct a new literal node.
    INode(QString lit, IWorld* world):
        world(world),
        lit(lit)
    {
    }

    /// Construct a new blank node with a private identifier.
    INode(IWorld* world) : world(world)
    {
    }

    /// Create a copy of given node. Note! both nodes share the same pointer.
    INode(INode* node, IWorld* world) : world(0)
    {
    }

    virtual ~INode()
    {
    }

    /// Return node's literal value or empty string.
    QString Lit() const {return lit;}

    /// Return resource node uri as QUrl object.
    QUrl Uri() const {return uri;}

    /// Return node type. Supported node types are Empty, Blank, Resource and Literal.
    NodeType Type() const {return type;}

    /// Return world object that owns given nodes.
    IWorld* World() const {return world;}

protected:
    QString blank_id;
    QString lit;
    QUrl uri;
    NodeType type;
    IWorld* world;
};