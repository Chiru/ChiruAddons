// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "RdfModuleFwd.h"
#include "IMemoryStore.h"
#include "CoreTypes.h"

#include <redland.h>
#include <QObject>
#include <QUrl>
#include <QVariant>
#include <QMap>

class RdfMemoryStore : public IMemoryStore
{
    Q_OBJECT

public:
    RdfMemoryStore(IWorld* world);
    virtual ~RdfMemoryStore();

public slots:
    /// Read rdf data from string and push it into RdfMemoryStore.
    /** @param Raw rdf data
        @return Return true if parse succeeded.*/
    bool FromString(QString data);

    /// Convert model data to string format.
    QString toString() const;

    /// Do rdf query using a given statement.
    QVariantList Select(IStatement* statement);

    /// Get model statements as array. Note! Model changes wont update on returned statments.
    QVariantList Statements();
    
    /// Add new statement to MemoryStore.
    /** @param statment statement that we want to remove.
        @return Return true if succeeded.*/
    bool AddStatement(IStatement* statement);

    /// Remove given statement from the MemoryStore.
    /** @param statment statement that we want to remove.
        @return Return true if succeeded.*/
    bool RemoveStatement(IStatement* statement);

    /// Create new clone of given IMemoryStore
    IMemoryStore *Clone();

private:
    librdf_storage* storage;
    librdf_model *model;
    ModelType type;
};
