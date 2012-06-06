// For conditions of distribution and use, see copyright notice in LICENSE

#include <stdio.h>

#include <QScriptValue>
#include <QScriptContext>
#include <QScriptEngine>
#include <QVariant>

#include "ScriptManager.h"
#include "qscript_tag.h" 

Q_DECLARE_METATYPE(CieMap::Tag*)
Q_DECLARE_METATYPE(CieMap::Tag)

void ToExistingScriptValue_tag(QScriptEngine *engine, const CieMap::Tag &value, QScriptValue obj)
{
    obj.setProperty("data", qScriptValueFromValue(engine, value.data), QScriptValue::Undeletable);
    obj.setProperty("type", qScriptValueFromValue(engine, value.type), QScriptValue::Undeletable);
}

static QScriptValue tag_tag(QScriptContext *context, QScriptEngine *engine)
{
    if (context->argumentCount() != 0) {
        printf("Error! Invalid number of arguments passed to function tag_tag in file %s, line %d!\nExpected 0, but got %d!\n", __FILE__, __LINE__, context->argumentCount());
        return QScriptValue();
    }
    CieMap::Tag ret;
    return qScriptValueFromValue(engine, ret);
}

static QScriptValue tag_tag_qstring(QScriptContext *context, QScriptEngine *engine)
{
    if (context->argumentCount() != 1) {
        printf("Error! Invalid number of arguments passed to function tag_tag_qstring in file %s, line %d!\nExpected 1, but got %d!\n", __FILE__, __LINE__, context->argumentCount());
        return QScriptValue();
    }
    CieMap::Tag ret;
    QString type = qscriptvalue_cast<QString>(context->argument(0));
    ret.SetType(type);
    return qScriptValueFromValue(engine, ret);
}

static QScriptValue tag_tag_qstring_qstring(QScriptContext *context, QScriptEngine *engine)
{
    if (context->argumentCount() != 2) {
        printf("Error! Invalid number of arguments passed to function tag_tag_qstring_qstring in file %s, line %d!\nExpected 2, but got %d!\n", __FILE__, __LINE__, context->argumentCount());
        return QScriptValue();
    }
    CieMap::Tag ret;
    QString type = qscriptvalue_cast<QString>(context->argument(0));
    QString data = qscriptvalue_cast<QString>(context->argument(1));
    ret.SetType(type);
    ret.SetData(data);
    return qScriptValueFromValue(engine, ret);
}

static QScriptValue tag_toString_const(QScriptContext *context, QScriptEngine *engine)
{
    CieMap::Tag This;
    // Qt oddity (bug?): Sometimes the built-in toString() function doesn't give us this from thisObject, but as the first argument.
    if (context->argumentCount() > 0)
        This = qscriptvalue_cast<CieMap::Tag>(context->argument(0));
    else
        This = qscriptvalue_cast<CieMap::Tag>(context->thisObject());
    QString ret = "Tag(" + This.type + "," + This.data + ")";
    return qScriptValueFromValue(engine, ret);
}

void FromScriptValue_tag(const QScriptValue &obj, CieMap::Tag &value)
{
    value.data = qScriptValueToValue<QString>(obj.property("data"));
    value.type = qScriptValueToValue<QString>(obj.property("type"));
}

QScriptValue ToScriptValue_tag(QScriptEngine *engine, const CieMap::Tag &value)
{
    // The contents of this variant are NOT used. The real data lies in the data() pointer of this QScriptValue. This only exists to enable overload resolution to work for QObject slots.
    QScriptValue obj = engine->newVariant(QVariant::fromValue(value));
    ToExistingScriptValue_tag(engine, value, obj);
    return obj;
}

static QScriptValue tag_ctor(QScriptContext *context, QScriptEngine *engine)
{
    if (context->argumentCount() == 0)
        return tag_tag(context, engine);
    if (context->argumentCount() == 1 && context->argument(0).isString())
        return tag_tag_qstring(context, engine);
    if (context->argumentCount() == 2 && context->argument(0).isString() && context->argument(1).isString())
        return tag_tag_qstring_qstring(context, engine);
    printf("tag_ctor failed to choose the right function to call! Did you use 'var x = Tag();' instead of 'var x = new Tag();'?\n");
    return QScriptValue();
}

QScriptValue register_tag_prototype(QScriptEngine *engine)
{
    QScriptValue proto = engine->newObject();
    proto.setProperty("toString", engine->newFunction(tag_toString_const, 0), QScriptValue::Undeletable | QScriptValue::ReadOnly);
    engine->setDefaultPrototype(qMetaTypeId<CieMap::Tag>(), proto);
    engine->setDefaultPrototype(qMetaTypeId<CieMap::Tag*>(), proto);
    qScriptRegisterMetaType(engine, ToScriptValue_tag, FromScriptValue_tag, proto);

    QScriptValue ctor = engine->newFunction(tag_ctor, proto, 2);
    engine->globalObject().setProperty("Tag", ctor, QScriptValue::Undeletable | QScriptValue::ReadOnly);

    return ctor;
}