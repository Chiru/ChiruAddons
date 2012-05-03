#pragma once

#include "3dUiModuleFwd.h"

#include <QObject>

class QString;
class QNetworkReply;

namespace CieMap
{
/// Utility functions and services that are available to custom scripts.
/// @todo Idea: expose as 'scriptServices' dynamic object to scripts?
class ScriptServices : public QObject
{
    Q_OBJECT

public:
    /// Sends request to a preprocessor web service
    /** @param dataPreprocessorUrl Url of the preprocessor web service
        @return Response from the preprocessor */
    static HttpRequestResponse *SendPreprocessorRequest(const QString &dataPreprocessorUrl)
    {
        return SendPreprocessorRequest(dataPreprocessorUrl, "", 0);
    }

    /// Sends request to a preprocessor web service
    /** @param dataPreprocessorUrl Url of the preprocessor web service
        @param requestService Service to use for sending the http request.
        @return Response from the preprocessor */
    static HttpRequestResponse *SendPreprocessorRequest(const QString &dataPreprocessorUrl, IHttpRequestService *requestService)
    {
        return SendPreprocessorRequest(dataPreprocessorUrl, "", requestService);
    }

    /// Sends request to a preprocessor web service
    /** @param dataPreprocessorUrl Url of the preprocessor web service
        @param dataSourceUri A data source url that should be passed on to the preprocessor.
        @return Response from the preprocessor */
    static HttpRequestResponse *SendPreprocessorRequest(const QString &dataPreprocessorUrl, const QString &dataSourceUri)
    {
        return SendPreprocessorRequest(dataPreprocessorUrl, dataSourceUri, 0);
    }

    /// Sends request to a preprocessor web service
    /** @param dataPreprocessorUrl Url of the preprocessor web service
        @param dataSourceUri A data source url that should be passed on to the preprocessor.
        @param requestService Service to use for sending the http request.
        @return Response from the preprocessor */
    static HttpRequestResponse *SendPreprocessorRequest(const QString &dataPreprocessorUrl, const QString &dataSourceUri, IHttpRequestService *requestService);

    /// Url encodes the specified string, using '+' for spaces
    static QString UrlEncode(QString url);

    /// Url encodes the specified string, using '%20' for spaces
    static QString UrlPathEncode(QString url);

    /// Use this function to distinguish the two types of geo location data (location name, and gps coordinates)
    /** Returns true if the string contains gps coordinates.
        @param data Geo location data
        @return True if the location data contains gps data, false otherwise */
    static bool IsGpsData(const QString &data);
};

/*internal sealed*/
class HttpRequest : public QObject
{
    Q_OBJECT
    // Q_PROPERTY(response
    // Q_PROPERTY(uri
    // Q_PROPERTY(postData
public:
    void SetResponse(HttpRequestResponse *value) { response = value; }
    HttpRequestResponse *Response() const { return response; }

    QString Uri() const { return uri; }
    void SetUri(const QString &value) { uri = value;}

    void SetPostData(const QString &value) { postData = value;}
    QString PostData() const;

    Q_INVOKABLE void SendRequest();

private:
    QByteArray ReadReply(QNetworkReply *reply);

    HttpRequestResponse *response;
    QString uri;
    QString postData;
};

}
