
//using SemWeb;

#include "HttpRequestResponse.h"

namespace CieMap
{

bool HttpRequestResponse::IsReady()
{
    // lock (this) TODO
    return ready;
}

SemWeb::MemoryStore *HttpRequestResponse::Data()
{
    // TODO lock (this)
        return requestResponse;
}

void HttpRequestResponse::SetResponse(const QByteArray/*byte[]*/ &response)
{
    //TODO
/*
    MemoryStore memoryStore = new MemoryStore();
    string error = null;

    using (StreamReader xmlStream = new StreamReader(new MemoryStream(response)))
    {
        try
        {
            //Console.Out.WriteLine(System.Text.Encoding.UTF8.GetString(response));
            memoryStore.Import(new RdfXmlReader(xmlStream));
        }
        catch (System.Xml.XmlException e)
        {
            error = e.Message;
        }
    }

    lock (this)
    {
        requestResponse = memoryStore;
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
