// For conditions of distribution and use, see copyright notice in LICENSE

#include "HttpRequestResponse.h"

namespace CieMap
{

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
    return "";
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