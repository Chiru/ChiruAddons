// For conditions of distribution and use, see copyright notice in LICENSE

#include "RdfNode.h"
#include "RdfWorld.h"
#include "RdfStatement.h"
#include "LoggingFunctions.h"

/// Construct a new resource node.
RdfNode::RdfNode(QUrl uri, IWorld* world) : INode(uri, world)
{
    type = Resource;
    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    assert(rdfWorld && "Failed to dynamic cast IWorld to RdfWorld");

    if (rdfWorld)
        node = librdf_new_node_from_uri_string(rdfWorld->world, (const unsigned char*)uri.toString().toUtf8().constData());
}

/// Construct a new literal node.
RdfNode::RdfNode(QString lit, IWorld* world) : INode(lit, world)
{
    type = Literal;
    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    assert(rdfWorld && "Failed to dynamic cast IWorld to RdfWorld");

    if (rdfWorld)
        node = librdf_new_node_from_literal(rdfWorld->world, (const unsigned char*)lit.toUtf8().constData(), 0, 0);
}

/// Construct a new blank node with a private identifier.
RdfNode::RdfNode(IWorld* world) : INode(world)
{
    type = Blank;
    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    assert(rdfWorld && "Failed to dynamic cast IWorld to RdfWorld");

    if (rdfWorld)
        node = librdf_new_node(rdfWorld->world);
    else
        LogError("Failed to cast IWorld to RdfWorld.");
}

RdfNode::RdfNode(INode* node, IWorld* world) : INode(node, world)
{
    RdfNode* rdfNode = dynamic_cast<RdfNode*>(node);
    assert(rdfNode && "Failed to dynamic cast INode pointer to RdfNode pointer.");
    if (rdfNode)
    {
        this->node = rdfNode->node;
        switch (librdf_node_get_type(this->node))
        {
        case LIBRDF_NODE_TYPE_BLANK:
            blank_id = QString::fromUtf8((const char *)librdf_node_get_blank_identifier(this->node));
            type = Blank;
            break;
        case LIBRDF_NODE_TYPE_RESOURCE:
            uri = QString::fromUtf8((const char *)librdf_uri_to_string(librdf_node_get_uri(this->node)));
            type = Resource;
            break;
        case LIBRDF_NODE_TYPE_LITERAL:
            lit = QString::fromUtf8((const char *)librdf_node_get_literal_value(this->node));
            type = Literal;
            break;
        default:
            type = Empty;
            break;
        }
    }
    else
        LogError("RdfNode:RdfNode(INode*) Failed to dynamic cast INode pointer to RdfNode pointer.");
}

RdfNode::RdfNode(librdf_node* node, IWorld* world) : INode(world)
{
    this->node = node;
    this->world = world;
    switch (librdf_node_get_type(node))
    {
    case LIBRDF_NODE_TYPE_BLANK:
        blank_id = QString::fromUtf8((const char *)librdf_node_get_blank_identifier(node));
        type = Blank;
        break;
    case LIBRDF_NODE_TYPE_RESOURCE:
        uri = QString::fromUtf8((const char *)librdf_uri_to_string(librdf_node_get_uri(node)));
        type = Resource;
        break;
    case LIBRDF_NODE_TYPE_LITERAL:
        lit = QString::fromUtf8((const char *)librdf_node_get_literal_value(node));
        type = Literal;
        break;
    default:
        type = Empty;
        break;
    }
}

RdfNode::~RdfNode()
{
    if (node)
        librdf_free_node(node);
}