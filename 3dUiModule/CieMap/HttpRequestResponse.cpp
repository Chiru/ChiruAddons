// For conditions of distribution and use, see copyright notice in LICENSE

#include "HttpRequestResponse.h"
#include "RdfModule.h"
#include "IWorld.h"
#include "IMemoryStore.h"

namespace CieMap
{

HttpRequestResponse::~HttpRequestResponse()
{
    if(requestResponse)
    {
        IWorld *world = RdfModule::WorldInstance();
        world->FreeStore(requestResponse);
    }
}

bool HttpRequestResponse::IsReady()
{
    // lock (this) TODO
    return ready;
}

IMemoryStore *HttpRequestResponse::Data()
{
    // TODO lock (this)
    return requestResponse;
}

void HttpRequestResponse::SetResponse(const QByteArray/*byte[]*/ &response)
{
    IMemoryStore *memoryStore = 0;
    if (!requestResponse)
    {
        IWorld *world = RdfModule::WorldInstance();
        memoryStore = world->CreateStore();
    }
    else
        memoryStore = requestResponse;
    QString error = "";

    QString str = QString::fromUtf8(response);
    if (!memoryStore->FromString(str))
    {
        error = "Failed to parse rdf data to memory store.";
    }

    requestResponse = memoryStore;
    ready = true;
    errorDescription = error;
    emit Ready(this);
}

QString HttpRequestResponse::Error()
{
    return errorDescription;
}

}
