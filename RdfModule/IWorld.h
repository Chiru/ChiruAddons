// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include <QUrl>

class IModel;
class INode;
class IStatement;

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

    virtual INode* CreateResource(QUrl uri) = 0;
    virtual INode* CreateNode() = 0;
    virtual INode* CreateLiteral(QString lit_v) = 0;

    virtual IStatement* CreateStatement(IStatement* statement) = 0;
    virtual IStatement* CreateStatement(INode* subject, INode* predicate, INode* object) = 0;
};

//Q_DECLARE_INTERFACE(IWorld, "Tundra.RdfModule.IWorld")