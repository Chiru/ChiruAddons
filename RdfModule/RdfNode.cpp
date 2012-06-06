// For conditions of distribution and use, see copyright notice in LICENSE

#include "RdfNode.h"
#include "RdfWorld.h"
#include "RdfStatement.h"
#include "LoggingFunctions.h"

RdfNode::RdfNode(QUrl uri, IWorld* world) : INode(uri, world)
{
    type = Resource;
    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    assert(rdfWorld && "Failed to dynamic cast IWorld to RdfWorld");

    if (rdfWorld)
    {
        node = librdf_new_node_from_uri_string (rdfWorld->world, (const unsigned char*)uri.toString().toUtf8().constData());
        rdfWorld->RegisterNode(this);
    }
}

RdfNode::RdfNode(QString lit, IWorld* world) : INode(lit, world)
{
    type = Literal;
    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    assert(rdfWorld && "Failed to dynamic cast IWorld to RdfWorld");

    if (rdfWorld)
    {
        node = librdf_new_node_from_literal(rdfWorld->world, (const unsigned char*)lit.toUtf8().constData(), 0, 0);
        rdfWorld->RegisterNode(this);
    }
    else
        LogError("Failed to cast IWorld to RdfWorld.");
}

RdfNode::RdfNode(IWorld* world) : INode(world)
{
    type = Blank;
    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    assert(rdfWorld && "Failed to dynamic cast IWorld to RdfWorld");

    if (rdfWorld)
    {
        node = librdf_new_node(rdfWorld->world);
        rdfWorld->RegisterNode(this);
    }
    else
        LogError("Failed to cast IWorld to RdfWorld.");
}

RdfNode::RdfNode(INode* node, IWorld* world) : INode(node, world)
{
    RdfNode* rdfNode = dynamic_cast<RdfNode*>(node);
    assert(rdfNode && "RdfNode:RdfNode(INode*) Failed to dynamic cast INode pointer to RdfNode pointer.");
    if (rdfNode)
        FromRawNode(rdfNode->node, world);
    else
        LogError("RdfNode:RdfNode(INode*) Failed to dynamic cast INode pointer to RdfNode pointer.");
}

RdfNode::RdfNode(librdf_node* node, IWorld* world) : INode(world)
{
    FromRawNode(node, world);
}

RdfNode::~RdfNode() 
{
    if (node) librdf_free_node(node);
}

void RdfNode::FromRawNode(librdf_node* node, IWorld* world)
{
    this->node = librdf_new_node_from_node(node);
    RdfWorld *w = dynamic_cast<RdfWorld *>(world);
    if (!w) LogError("RdfNode::FromRawNode: Failed to dynamic cast IWorld to RdfWorld.");
    assert(w && "RdfNode::FromRawNode: Failed to dynamic cast IWorld to RdfWorld.");
    w->RegisterNode(this);

    this->world = world;
    switch (librdf_node_get_type(node))
    {
    case LIBRDF_NODE_TYPE_BLANK:
        //\todo make sure that setting identifier to null wont break anything --Joosua.
        blank_id = QString::fromUtf8((const char *)librdf_node_get_blank_identifier(this->node));
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
        blank_id = QString::fromUtf8((const char *)librdf_node_get_blank_identifier(this->node));
        type = Empty;
        break;
    }
}