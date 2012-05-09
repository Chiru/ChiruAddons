// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "CoreTypes.h"
#include "RdfModuleFwd.h"

#include <QUrl>
#include <QVariant>
#include <QObject>

class IMemoryStore : public QObject
{
    Q_OBJECT
    Q_PROPERTY(ModelType type READ Type)
    Q_ENUMS(IMemoryStore::ModelType)

public:
    enum ModelType
    {
        None   = 0,
        RdfXml = 1
    };

    IMemoryStore(IWorld* world) : type(None),
        world(world)
    {
    }

    virtual ~IMemoryStore()
    {
    }

    ModelType Type() const {return type;}

    virtual bool FromUri(QUrl uri) = 0;
    virtual bool FromString(QString data) = 0;

    virtual QString toString() const = 0;

    virtual QVariantList Select(IStatement* statement) = 0;
    virtual QVariantList Statements() = 0;

    virtual bool AddStatement(IStatement* statement) = 0;
    virtual bool RemoveStatement(IStatement* statement) = 0;

protected:
    IWorld* world;
    ModelType type;
};