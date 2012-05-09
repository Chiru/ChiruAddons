#pragma once

#include <QObject>

class IModel;
class IWorld;

class RdfMemoryStore : public QObject
{
    Q_OBJECT
    Q_PROPERTY(IModel* model READ Model)

public:
    RdfMemoryStore(IWorld* world);
    virtual ~RdfMemoryStore();

    IModel* Model() const { return model; }

private:
    IModel* model;
};