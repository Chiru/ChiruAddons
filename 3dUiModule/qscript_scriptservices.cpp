// For conditions of distribution and use, see copyright notice in LICENSE

#include "CieMap/HttpRequestService.h"
#include "CieMap/HttpRequestResponse.h"
#include "CieMap/ScriptServices.h"

#include <QScriptValue>
#include <QScriptContext>
#include <QScriptEngine>
#include <QVariant>

#include <stdio.h>

Q_DECLARE_METATYPE(CieMap::HttpRequestResponse*)
Q_DECLARE_METATYPE(CieMap::HttpRequestService*)

template<typename T>
bool QSVIsOfType(const QScriptValue &value)
{
    // For the math classes, we expose the actual type as a member property, since we are not using
    // the opaque QVariant-based interop.
    // For basic types, like float and int, the latter value.toVariant().canConvert<T> is used.
    return (value.prototype().property("metaTypeId").toInt32() == qMetaTypeId<T>() || value.toVariant().canConvert<T>());
}

static QScriptValue scriptservices_httprequestresponse_qsting_qstring_httprequest(QScriptContext *context, QScriptEngine *engine)
{
    if (context->argumentCount() != 3) { printf("Error! Invalid number of arguments passed to function scriptservices_httprequestresponse_qsting_qstring_httprequest in file %s, line %d!\nExpected 3, but got %d!\n", __FILE__, __LINE__, context->argumentCount()); return QScriptValue(); }
    QString url = context->argument(0).toString();
    QString data = context->argument(1).toString();
    CieMap::HttpRequestService *request = qscriptvalue_cast<CieMap::HttpRequestService *>(context->argument(2));
    CieMap::HttpRequestResponse *response = CieMap::ScriptServices::SendPreprocessorRequest(url, data, request);
    return qScriptValueFromValue(engine, response);
}

static QScriptValue scriptservices_sendpreprocessorrequest(QScriptContext *context, QScriptEngine *engine)
{
    if (context->argumentCount() == 3 && context->argument(0).isString() && context->argument(1).isString() && context->argument(2).isQObject())
        return scriptservices_httprequestresponse_qsting_qstring_httprequest(context, engine);
    return QScriptValue();
}

QScriptValue register_scriptservice_prototype(QScriptEngine *engine)
{
    QScriptValue proto = engine->newObject();
    proto.setProperty("SendPreprocessorRequest", engine->newFunction(scriptservices_sendpreprocessorrequest, 3), QScriptValue::Undeletable | QScriptValue::ReadOnly);
    engine->globalObject().setProperty("ScriptServices", proto, QScriptValue::Undeletable | QScriptValue::ReadOnly);

    return proto;
}
