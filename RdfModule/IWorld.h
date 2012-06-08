// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "RdfModuleFwd.h"

#include <QUrl>
#include <QSet>

class QObject;

class IWorld : public QObject
{
    Q_OBJECT

public:
    IWorld()
    {
    }

    virtual ~IWorld()
    {
    }

    /// Create a new instance of rdf Model/Store object.
    virtual IMemoryStore* CreateStore() = 0; 

    virtual void FreeStore(IMemoryStore *store) = 0;

    /// Create new resource node for given uri value (statment's subject and predicate).
    virtual INode*  CreateResource(QUrl uri) = 0;

    /// Create blank node.
    virtual INode*  CreateNode() = 0;

    /// Create literal node (statment's object).
    virtual INode*  CreateLiteral(QString lit_v) = 0;

    virtual void FreeNode(INode *node) = 0;

    /// Create new statment for given subject, predicate and object.
    /*  Note! Node pointers aren't stored to a statement, instead their values get copied and.
        inserted in new node objects, this way the memory management gets much simpler.*/
    virtual IStatement* CreateStatement(INode* subject, INode* predicate, INode* object) = 0;

    virtual void FreeStatement(IStatement *statement) = 0;
};