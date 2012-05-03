// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include <redland.h>
#include <QObject>
#include <QUrl>
#include <QVariant>
#include <QMap>

#include "CoreTypes.h"
#include "RdfModuleFwd.h"

#include "IModel.h"

class RdfStatement;

class RdfXmlModel : public IModel
{
    Q_OBJECT

public:
    RdfXmlModel(IWorld* world);
    virtual ~RdfXmlModel();

public slots:
    bool FromUri(QUrl uri);
    bool FromString(QString data);

    QString toString() const;

    QVariantList Select(IStatement* statement);
    QVariantList Statements();

    bool AddStatement(IStatement* statement);
    bool RemoveStatement(IStatement* statement);

private:
    librdf_model *model;
    ModelType type;
};