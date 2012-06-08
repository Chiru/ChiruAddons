// For conditions of distribution and use, see copyright notice in LICENSE

#include <stdio.h>

#include <QScriptValue>
#include <QScriptContext>
#include <QScriptEngine>
#include <QVariant>

#include "HttpRequestService.h"
#include "HttpRequestResponse.h"
#include "ScriptServices.h"

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
    //if (context->argumentCount() != 1) { printf("Error! Invalid number of arguments passed to function float2_At_int in file %s, line %d!\nExpected 1, but got %d!\n", __FILE__, __LINE__, context->argumentCount()); PrintCallStack(context->backtrace()); return QScriptValue(); }
    QString str = context->argument(0).toString();
    QString str2 = context->argument(1).toString();
    CieMap::HttpRequestService *request = qscriptvalue_cast<CieMap::HttpRequestService *>(context->argument(2));
    /*float2 This = qscriptvalue_cast<float2>(context->thisObject());
    int index = qscriptvalue_cast<int>(context->argument(0));
    float & ret = This.At(index);
    ToExistingScriptValue_float2(engine, This, context->thisObject());*/
    CieMap::HttpRequestResponse *response = CieMap::ScriptServices::SendPreprocessorRequest(str, str2, request);
    return qScriptValueFromValue(engine, response);
}

static QScriptValue scriptservices_sendpreprocessorrequest(QScriptContext *context, QScriptEngine *engine)
{
    if (context->argumentCount() == 3 && context->argument(0).isString() && context->argument(1).isString() && context->argument(2).isQObject()/*QSVIsOfType<CieMap::HttpRequestService*>(context->argument(2))*/)
        return scriptservices_httprequestresponse_qsting_qstring_httprequest(context, engine);
    /*if (context->argumentCount() == 1 && QSVIsOfType<int>(context->argument(0)))
        return float2_At_int_const(context, engine);
    printf("float2_At_selector failed to choose the right function to call in file %s, line %d!\n", __FILE__, __LINE__); PrintCallStack(context->backtrace()); return QScriptValue();*/
    return QScriptValue();
}

QScriptValue register_scriptservice_prototype(QScriptEngine *engine)
{
    QScriptValue proto = engine->newObject();
    proto.setProperty("SendPreprocessorRequest", engine->newFunction(scriptservices_sendpreprocessorrequest, 3), QScriptValue::Undeletable | QScriptValue::ReadOnly);
    engine->globalObject().setProperty("ScriptServices", proto, QScriptValue::Undeletable | QScriptValue::ReadOnly);

    return proto;
}