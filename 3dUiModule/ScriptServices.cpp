// For conditions of distribution and use, see copyright notice in LICENSE

// TODO using System.Threading;
//using SemWeb; TODO

#include "ScriptServices.h"

#include <QByteArray>
#include <QStringList>

#include "RdfModule.h"
#include "IWorld.h"
#include "IMemoryStore.h"
#include "LoggingFunctions.h"
#include "INode.h"
#include "IStatement.h"
#include "IHttpRequestService.h"
#include "HttpRequestResponse.h"

namespace CieMap
{

HttpRequestResponse *ScriptServices::SendPreprocessorRequest(
    const QString &dataPreprocessorUrl,
    const QString &dataSourceUri,
    IHttpRequestService *requestService)
{
    QString uri = dataPreprocessorUrl;
    if (dataPreprocessorUrl.isEmpty())
    {
        /// @todo Print error throw new System.ArgumentException("Parameter cannot be null", "dataPreprocessorUri");
        LogError("ScriptServices::SendPreprocessorRequest: Url parameter cannot be null");
        return 0;
    }

    QString encodedSourceUri = dataSourceUri;
    if (!encodedSourceUri.isEmpty())
        encodedSourceUri = UrlEncode(dataSourceUri);

    IWorld *world       = RdfModule::WorldInstance();
    IMemoryStore *store = world->CreateStore();
    INode* from         = world->CreateResource(QUrl("http://cie/news#"));//dataPreprocessorUrl + "?s=" + encodedSourceUri));
    INode* dataSource   = world->CreateResource(QUrl("http://cie/data-source")); //todo RDFVocabulary.dataSource
    INode* liter        = world->CreateLiteral(dataSourceUri);
    IStatement* dataSourceStamement = world->CreateStatement(from, dataSource, liter);

    store->AddStatement(dataSourceStamement);
    QString postData = store->toString();

    world->FreeNode(from);
    world->FreeNode(dataSource);
    world->FreeNode(liter);
    world->FreeStatement(dataSourceStamement);
    world->FreeStore(store);

    HttpRequestResponse* response;
    if (requestService != 0)
    {
        response = requestService->SendHttpRequest(uri, postData);
    }
    else
    {
        response = new HttpRequestResponse();

        HttpRequest *searchRequest = new HttpRequest();// { Response = response, Uri = uri, PostData = postData};
        searchRequest->SetResponse(response);
        searchRequest->SetUri(uri);
        searchRequest->SetPostData(postData);

        searchRequest->SendRequest();
        /*Thread thread = new Thread(new ThreadStart(searchRequest.SendRequest)) { IsBackground = true };
        thread.Start();*/

        return response;
    }

    return response;
    // TODO
/*
    QString uri = dataPreprocessorUrl;

    if (dataPreprocessorUrl.isEmpty())
    {
        /// @todo Print error throw new System.ArgumentException("Parameter cannot be null", "dataPreprocessorUri");
        return;
    }

    QString encodedSourceUri = dataSourceUri;
    if (!encodedSourceUri.isEmpty())
        encodedSourceUri = UrlEncode(dataSourceUri);

    MemoryStore store = new MemoryStore();

    // 'news' is the identifier for the preprocessor
    Entity from = new Entity(dataPreprocessorUrl + "?s=" + encodedSourceUri);
    Entity dataSource = new Entity(RDFVocabulary.dataSource);
    Statement dataSourceStamement = new Statement(from, dataSource, new Literal(dataSourceUri));
    store.Add(dataSourceStamement);

    StringBuilder sb = new StringBuilder();
    using (RdfWriter writer = new RdfXmlWriter(new StringWriter(sb)))
    {
        writer.Namespaces.AddNamespace(RDFVocabulary.baseUri, RDFVocabulary.namespacePrefix);
        writer.Write(store);
    }
    string postData = sb.ToString();

    HttpRequestResponse response;
    if (requestService != null)
    {
        response = requestService.SendHttpRequest(uri, postData);
    }
    else
    {
        response = new HttpRequestResponse();

        HttpRequest searchRequest = new HttpRequest() { Response = response, Uri = uri, PostData = postData};

        Thread thread = new Thread(new ThreadStart(searchRequest.SendRequest)) { IsBackground = true };
        thread.Start();

        return response;
    }

    return response;
*/
}

QString ScriptServices::UrlEncode(QString url)
{
    //return System.Web.HttpUtility.UrlEncode(url);
    url.replace(" ", "+");
    return url;
}

QString ScriptServices::UrlPathEncode(QString url)
{
    //return System.Web.HttpUtility.UrlPathEncode(url);
    ///@todo How about url.toUtf8().toPercentEncoding();
    url.replace(" ", "%20");
    return url;
}

bool ScriptServices::IsGpsData(const QString &data)
{
    QStringList components = data.split(',', QString::SkipEmptyParts);
    if (components.size() != 3)
        return false;

    foreach(const QString &c, components)
    {
        bool ok;
        c.toFloat(&ok);
        if (!ok)
            return false;
    }

    return true;
}

void HttpRequest::SendRequest()
{
    /*QNetworkRequest *request;
    QNetworkAccessManager *manager;*/

    ///@todo Implement with Qt HTTP stuff
/*
    networkAccessManager->put
    networkAccessManager->get
    networkAccessManager->post
    networkAccessManager = new QNetworkAccessManager();
    connect(networkAccessManager, SIGNAL(finished(QNetworkReply*)), SLOT(OnHttpTransferFinished(QNetworkReply*)));

*/
/*
    if (String.IsNullOrEmpty(Uri))
        throw new System.NullReferenceException("Uri cannot be null");
    if (String.IsNullOrEmpty(PostData))
        throw new System.NullReferenceException("PostData cannot be null");
    
    HttpWebRequest request = (HttpWebRequest)WebRequest.Create(Uri);
    request.AuthenticationLevel = System.Net.Security.AuthenticationLevel.None;
    request.Method = "POST";
    request.ContentType = "application/x-www-form-urlencoded";
    string data = "data=" + PostData;
    byte[] postBytes = Encoding.UTF8.GetBytes(data);
    request.ContentLength = postBytes.Length;
    using (Stream requestStream = request.GetRequestStream())
        requestStream.Write(postBytes, 0, postBytes.Length);

    HttpWebResponse webResponse;

    try
    {
        webResponse = (HttpWebResponse)request.GetResponse();

        if (webResponse.StatusCode == HttpStatusCode.OK)
        {
            Response.SetResponse(ReadReply(webResponse));
        }
        else
        {
            Response.Error = webResponse.StatusDescription;
        }

        webResponse.Close();
    }
    catch (WebException e)
    {
        Response.Error = e.Message;
    }
*/
}

QByteArray HttpRequest::ReadReply(QNetworkReply *reply)
{
    // TODO
    return 0;
/*
    Stream resStream = webResponse.GetResponseStream();

    int count = 0;
    byte[] readbuffer = new byte[HttpRequestResponse.MaxSize];

    count = resStream.Read(readbuffer, 0, readbuffer.Length);
    byte[] buffer = new byte[count];
    Buffer.BlockCopy(readbuffer, 0, buffer, 0, count);

    resStream.Close();

    return buffer;
*/
}

}
