// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "CieMapFwd.h"
#include <QObject>

class QString;

namespace CieMap
{
/// Interface for a service that offers http search requests. Needed when normal dotNet http requests don't work,
/// f.ex. in Android devices.
class IHttpRequestService : public QObject
{
    Q_OBJECT

public:
    /// Sends and http get request with the specified uri and post data. Should use 'POST' method.
    /** @param url Url of the web service that receives the request
        @param postData POST data
        @return The response can be used to query the status and response of the request */
    virtual HttpRequestResponse *SendHttpRequest(const QString &url, const QString &postData) = 0;
};

}
