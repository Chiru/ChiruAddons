// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include <QObject>
#include <redland.h>

#include "IWorld.h"

class RdfNode;
class RdfStatement;

class RdfWorld :  public IWorld
{
    Q_OBJECT

public:
    RdfWorld();
    ~RdfWorld();

public slots:
    virtual INode* CreateResource(QUrl uri);
    virtual INode* CreateNode();
    virtual INode* CreateLiteral(QString lit_v);

    virtual IStatement* CreateStatement(IStatement* statement);
    virtual IStatement* CreateStatement(INode* subject, INode* predicate, INode* object);

public:
    librdf_world*  world;
    librdf_storage* storage;
    librdf_parser* parser;
    librdf_serializer* serializer;
};