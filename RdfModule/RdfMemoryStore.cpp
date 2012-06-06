// For conditions of distribution and use, see copyright notice in LICENSE

#include "RdfMemoryStore.h"
#include "RdfWorld.h"
#include "RdfStatement.h"

#include "LoggingFunctions.h"

Q_DECLARE_METATYPE(RdfStatement *)

RdfMemoryStore::RdfMemoryStore(IWorld* world) : IMemoryStore(world),
    type(RdfXml),
    model(0),
    storage(0)
{
    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    assert(rdfWorld && "Failed to dynamic cast IWorld to RdfWorld");

    storage = librdf_new_storage(rdfWorld->world, "memory", NULL, NULL);

    if (rdfWorld)
    {
        model = librdf_new_model(rdfWorld->world, storage, "");
        rdfWorld->RegisterStore(this);
    }
    else
        LogError("Failed to cast IWorld to RdfWorld.");
   
}

RdfMemoryStore::~RdfMemoryStore()
{
    if (model)
        librdf_free_model(model);
    if (storage)
        librdf_free_storage(storage);
}

bool RdfMemoryStore::FromString(QString data)
{
    RdfWorld* rdfWorld = dynamic_cast<RdfWorld*>(world);
    assert(rdfWorld && "Failed to dynamic cast IWorld to RdfWorld");

    if (rdfWorld)
    {
        // Note! Order to get this work, we need to intialize a dummy uri object. --Joosua.
        librdf_uri* uri=librdf_new_uri(rdfWorld->world, (const unsigned char*)"dummy");
        bool succeeded = !librdf_parser_parse_string_into_model(rdfWorld->parser, (const unsigned char*)data.toUtf8().constData(), uri, model) ? true : false;
        librdf_free_uri(uri);
        return succeeded;
    }
    LogWarning("Failed to parse string into a model.");
    return false;
}

QString RdfMemoryStore::toString() const
{
    QString str = "";
    unsigned char* c = librdf_model_to_string(model, 0, 0, 0, 0);
    str = QString::fromUtf8(reinterpret_cast<char*>(c));
    return str;
}

QVariantList RdfMemoryStore::Select(IStatement* statement)
{
    RdfStatement* rdfStatement = dynamic_cast<RdfStatement*>(statement);

    QVariantList statements;
    if (rdfStatement)
    {
        librdf_stream* stream = librdf_model_find_statements(model, rdfStatement->statement);
        while(!librdf_stream_end(stream))
        {
            librdf_statement *s = librdf_stream_get_object(stream);
            if (s) 
                statements << QVariant::fromValue<RdfStatement*>(new RdfStatement(world, s));
            librdf_stream_next(stream);
        }
        librdf_free_stream(stream);
    }
    else
    {
        LogWarning("Model::Select failed: Null statement.");
    }
    return statements;
}

QVariantList RdfMemoryStore::Statements()
{
    librdf_stream* stream = librdf_model_as_stream(model);
    QVariantList statements;
    while(!librdf_stream_end(stream))
    {
        librdf_statement *s = librdf_stream_get_object(stream);
        if (s)
            statements << QVariant::fromValue<RdfStatement*>(new RdfStatement(world, s));
        librdf_stream_next(stream);
    }
    return statements;
}

bool RdfMemoryStore::AddStatement(IStatement* statement)
{
    if (statement->IsValid())
    {
        LogWarning("Only legal statements can be added to the model. The statment is legal when: subject=URI/blank, predicate=URI and object=literal/blank");
        return false;
    }

    RdfStatement* rdfStatement = dynamic_cast<RdfStatement*>(statement);
    assert(rdfStatement && "Failed to dynamic cast IStatement to RdfStatement");
    if (rdfStatement)
        if (!librdf_model_add_statement(model, rdfStatement->statement))
            return true;
    LogWarning("Failed to add a new statement into the model.");
    return false;
} 

bool RdfMemoryStore::RemoveStatement(IStatement* statement)
{
    if (statement->IsValid())
    {
        LogWarning("Only legal statements can be removed from the model. The statment is legal when: subject=URI/blank, predicate=URI and object=literal/blank");
        return false;
    }

    RdfStatement* rdfStatement = dynamic_cast<RdfStatement*>(statement);
    assert(rdfStatement && "Failed to dynamic cast IStatement to RdfStatement");
    if (rdfStatement)
        if (!librdf_model_remove_statement(model, rdfStatement->statement))
            return true;
    LogWarning("Failed to remove a statement from the model.");
    return false;
}