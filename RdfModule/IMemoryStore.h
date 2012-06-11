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
    Q_ENUMS(ModelType)

public:
    enum ModelType
    {
        None = 0,
        RdfXml = 1
    };

    IMemoryStore(IWorld* world) : type(None),
        world(world)
    {
    }

    virtual ~IMemoryStore()
    {
    }

    ModelType Type() const { return type; }

    /// Read rdf data from string and push it to RdfMemoryStore.
    /** @param Data rdf data
        @return Return true if parse succeeded.*/
    virtual bool FromString(QString data) = 0;

    /// Convert model data to string format.
    virtual QString toString() const = 0;

    /// Do rdf query using a given statement.
    /** @todo Replace QVariantList with StatementStream object --Joosua.
        @note It's up to user to release Statements found in QVariantList. */
    virtual QVariantList Select(IStatement* statement) = 0;

    /// Get model statements as array. Note! Model changes wont show on returned statments.
    virtual QVariantList Statements() = 0;

    /// Add new statement to MemoryStore.
    virtual bool AddStatement(IStatement* statement) = 0;

    /// Remove given statement from the MemoryStore.
    virtual bool RemoveStatement(IStatement* statement) = 0;

    /// Create new clone of given IMemoryStore
    virtual IMemoryStore *Clone() = 0;

    IWorld* World() const { return world; }

protected:
    IWorld* world;
    ModelType type;
};
