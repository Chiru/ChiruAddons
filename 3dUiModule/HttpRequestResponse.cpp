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

    QString str(response);
    if (!memoryStore->FromString(str))
    {
        error = "Failed to parse rdf data to memory store.";
    }

    requestResponse = memoryStore;
    ready = true;
    errorDescription = error;
    emit Ready(this);
    /*IWorld *world = IWorld::Instance();
    IMemoryStore *memoryStore = world->CreateStore();*/
    //TODO
/*
    IMemoryStore memoryStore = new IMemoryStore();
    string error = null;

    using (StreamReader xmlStream = new StreamReader(new MemoryStream(response)))
    {
        try
        {
            //Console.Out.WriteLine(System.Text.Encoding.UTF8.GetString(response));
            IMemoryStore.Import(new RdfXmlReader(xmlStream));
        }
        catch (System.Xml.XmlException e)
        {
            error = e.Message;
        }
    }

    lock (this)
    {
        requestResponse = RdfMemoryStore;
        ready = true;
        Error = error;
    }
*/
}

QString HttpRequestResponse::Error()
{
    return errorDescription;
    // TODO
    /*

    get
    {
        lock (this)
            return errorDescription;
    }

    set
    {
        lock (this)
        {
            errorDescription = value;
            ready = true;
        }
    }
    */
}

}
