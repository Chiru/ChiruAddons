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
    response(0)
{
    manager = new QNetworkAccessManager(this);
    connect(manager, SIGNAL(finished(QNetworkReply*)), this, SLOT(RequestResponse(QNetworkReply *)), Qt::UniqueConnection);
}

HttpRequestService::~HttpRequestService()
{
    SAFE_DELETE(manager);
}

HttpRequestResponse *HttpRequestService::SendHttpRequest(const QString &url, const QString &postData)
{
    if (!response)
        response = new HttpRequestResponse();

    QNetworkRequest request;
    request.setRawHeader("User-Agent", "realXtend Tundra");
    request.setUrl(QUrl(url));
    manager->post(request, "data=" + postData.toAscii());

    return response;
}

void HttpRequestService::RequestResponse(QNetworkReply *reply)
{
    // TODO check if response is valid.
    response->SetResponse(reply->readAll());
}

}
