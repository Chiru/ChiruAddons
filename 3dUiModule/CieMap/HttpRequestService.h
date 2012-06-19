// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "CieMapFwd.h"
#include "IHttpRequestService.h"
#include <QByteArray>
#include <QNetworkAccessManager>

class QString;
class QNetworkAccessManager;
class QNetworkReply;

namespace CieMap
{
class HttpRequestService : public IHttpRequestService
{
    Q_OBJECT
    Q_PROPERTY(QNetworkAccessManager::Operation operation READ Oreration WRITE SetOperation)

public:

    HttpRequestService();
    virtual ~HttpRequestService();

    virtual HttpRequestResponse *SendHttpRequest(const QString &url, const QString &data);

    QNetworkAccessManager::Operation Oreration() const;
    void SetOperation(QNetworkAccessManager::Operation operation);

signals:
    void RequestFinished(HttpRequestResponse *response);

private slots:
    void RequestResponse(QNetworkReply *reply);

private:
    QNetworkAccessManager *manager;
    HttpRequestResponse *response;
    QNetworkAccessManager::Operation operation;
};

}
