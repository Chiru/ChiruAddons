// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "3dUiModuleFwd.h"

#include <QObject>
#include <QString>

class QByteArray;

namespace CieMap
{
/// Response to a http search request.
/** An instance of this class is returned when initiating a http search request.
    When implementing Http request service with IHttpRequestService interface, create and return
    a new instance of this class and set response and error messages appropriately once the request
    has completed.

    Use IsReady to check the status of the request or connect to Ready signal. */
class HttpRequestResponse : public QObject
{
    Q_OBJECT
    Q_PROPERTY(IMemoryStore* data READ Data)
    Q_PROPERTY(bool ready READ IsReady)
    Q_PROPERTY(QString error READ Error)

public:
    HttpRequestResponse() : QObject(), ready(false), requestResponse(0) {}
    virtual ~HttpRequestResponse();

    /// Maximum size of the response. If the response is larger, the remainder is cut from the response.
    enum { MaxSize = 262144 };

    /// Is set to true when a response has been received to the request and the response can be read.
    bool IsReady();

    /// The response to the request.
    /** Only read after IsReady has been set to true.
        If there was an error during the request, this is set to the description of the error. */
    IMemoryStore *Data();

    /// Set the response from preprocessors.
    /** The response should be RdfXml data in a bytearray (UTF-8)
        If there is an error parsing the response, sets Error to the error message
        @note Sets IsReady to true.
        @param response RdfXml data that will be parsed. */
    void SetResponse(const QByteArray &response); /*byte[]*/

    /// After IsReady has been set to true, contains an error message if an error occurred during http request.
    /** This is set to null if no error occurred.
        @note Setting the error also sets IsReady to true */
    QString Error();

signals:
    /// Trigger ready signal when http request response is received.
    /** @param self signal sender.
     */
    void Ready(CieMap::HttpRequestResponse *self);

private:
    QString errorDescription;
    bool ready;
    IMemoryStore *requestResponse;
};

}
