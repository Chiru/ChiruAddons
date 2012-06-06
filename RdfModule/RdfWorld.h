// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "RdfModuleFwd.h"
#include "IWorld.h"

#include <QObject>
#include <redland.h>

class RdfWorld :  public IWorld
{
    Q_OBJECT

public:
    RdfWorld();
    ~RdfWorld();

    void RegisterNode(RdfNode* node);
    void RegisterStatement(RdfStatement* statement);
    void RegisterStore(RdfMemoryStore* store);

public slots:
    virtual IMemoryStore* CreateStore(); 
    virtual void FreeStore(IMemoryStore *store);

    virtual INode* CreateResource(QUrl uri);
    virtual INode* CreateNode();
    virtual INode* CreateLiteral(QString lit_v);
    virtual void FreeNode(INode *node);

    virtual IStatement* CreateStatement(INode* subject, INode* predicate, INode* object);
    virtual void FreeStatement(IStatement *statement);

private slots:
    void HandleNodeDestroy(QObject *obj);
    void HandleStatementDestroy(QObject *obj);
    void HandleStoreDestroy(QObject *obj);
public:
    librdf_world*  world;
    librdf_parser* parser;
    librdf_serializer* serializer;
};
