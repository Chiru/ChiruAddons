// For conditions of distribution and use, see copyright notice in LICENSE

#include "HttpRequestService.h"
#include "HttpRequestResponse.h"

#include "LoggingFunctions.h"
#include "CoreDefines.h"

#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>

namespace CieMap
{

HttpRequestService::HttpRequestService():
    IHttpRequestService(),
    manager(0),
    response(0),
    operation(QNetworkAccessManager::PostOperation)
{
    manager = new QNetworkAccessManager(this);
    connect(manager, SIGNAL(finished(QNetworkReply*)), this, SLOT(RequestResponse(QNetworkReply *)), Qt::UniqueConnection);
}

HttpRequestService::~HttpRequestService()
{
    SAFE_DELETE(manager);
}

HttpRequestResponse *HttpRequestService::SendHttpRequest(const QString &url, const QString &data)
{
    if (!response)
        response = new HttpRequestResponse();

    QNetworkRequest request;
    request.setRawHeader("User-Agent", "realXtend Tundra");
    if (operation == QNetworkAccessManager::PostOperation)
    {
        request.setUrl(QUrl(url));
        manager->post(request, "data=" + data.toAscii());
    }
    else if (operation == QNetworkAccessManager::GetOperation)
    {
        QUrl urlObj(url);
        urlObj.addQueryItem("data", data);
        request.setUrl(urlObj);
        manager->get(request);
    }
    else
        LogWarning("HttpRequestService(const QString &, const QString &): Only GET & POST operations are supported.");

    return response;
}

QNetworkAccessManager::Operation HttpRequestService::Oreration() const
{
    return operation;
}

void HttpRequestService::SetOperation(QNetworkAccessManager::Operation o)
{
    if (operation == QNetworkAccessManager::PostOperation || operation == QNetworkAccessManager::GetOperation)
        operation = o;
    else
        LogWarning("SetOperation(QNetworkAccessManager::Operation): Ignore change, only GET & POST operations are supported.");
}

void HttpRequestService::RequestResponse(QNetworkReply *reply)
{
    // TODO check if response is valid.
    response->SetResponse(reply->readAll());
}

}
