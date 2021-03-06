// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "RdfModuleFwd.h"
#include "CoreTypes.h"
#include "INode.h"

#include <redland.h>
#include <raptor.h>
#include <QObject>
#include <QUrl>

class RdfNode : public INode
{
    Q_OBJECT

public:
    /// Construct a new resource node.
    RdfNode(QUrl uri, IWorld* world);
    /// Construct a new literal node.
    RdfNode(QString lit, IWorld* world);
    /// Construct a new blank node with a private identifier.
    RdfNode(IWorld* world);
    /// Create a copy of given node.
    RdfNode(INode* node, IWorld* world);
    RdfNode(librdf_node* node, IWorld* world);

    virtual ~RdfNode();

    void SetLiteral(QString value);
    void SetUri(QUrl value);
    void SetType(NodeType value);

    librdf_node* node;
private:
    void FromRawNode(librdf_node* node, IWorld* world);
};
