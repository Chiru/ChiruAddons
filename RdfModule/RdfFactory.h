// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once
#include <QObject>
#include <QSet>
#include "CoreTypes.h"
#include "RdfModuleApi.h"

class IModel;
class IWorld;

class RDF_MODULE_API RdfFactory : public QObject
{
    Q_OBJECT

public:
    RdfFactory(){};
    virtual ~RdfFactory(){};

public slots:
    IModel* CreateModel(IWorld* world);
    IWorld* CreateWorld();

private:
    void AddObjectToStorage(QObject *obj)
    {
        storage.insert(obj);
    }

    typedef QSet<QObject*> QObjectStorage;
    QObjectStorage storage;
};