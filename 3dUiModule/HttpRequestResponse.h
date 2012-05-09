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

    Use IsReady to check the status of the request. */
class HttpRequestResponse : public QObject
{
    Q_OBJECT

public:
    HttpRequestResponse() : ready(false), requestResponse(0) {}

    /// Maximum size of the response. If the response is larger, the remainder is cut from the response.
    enum { MaxSize = 262144 };

    /// Is set to true when a response has been received to the request and the response can be read.
    bool IsReady();

    /// The response to the request.
    /** Only read after IsReady has been set to true.
        If there was an error during the request, this is set to the description of the error. */
    RdfMemoryStore *Data();

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

private:
    QString errorDescription;
    bool ready;
    RdfMemoryStore *requestResponse;
};

}
