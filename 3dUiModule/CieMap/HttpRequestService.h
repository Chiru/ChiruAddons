// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "CieMapFwd.h"
#include "IHttpRequestService.h"
#include <QByteArray>

class QString;
class QNetworkAccessManager;
class QNetworkReply;

namespace CieMap
{
class HttpRequestService : public IHttpRequestService
{
    Q_OBJECT

public:
    HttpRequestService();
    virtual ~HttpRequestService();

    virtual HttpRequestResponse *SendHttpRequest(const QString &url, const QString &postData);

signals:
    void RequestFinished(HttpRequestResponse *response);

private slots:
    void RequestResponse(QNetworkReply *reply);

private:
    QNetworkAccessManager *manager;
    HttpRequestResponse *response;
};

}
