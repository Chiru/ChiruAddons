#pragma once

#include <QString>

namespace CieMap
{
/// RDF vocabulary that is used both internally and when communicating with preprocessor.
struct RdfVocabulary
{
    /// Base uri
    static const QString baseUri;
    /// namespace prefix
    static const QString namespacePrefix;
    /// The preprocessor name
    static const QString sourceApplication;
    /// Geolocation, a 3D position vector or a name of a place
    static const QString geoLocation;
    /// The date of the source data and preprocessing
    static const QString dateTime;
    /// Preprocessed data
    static const QString data;
    /// Metadata for the preprocessed data
    static const QString metadata;
    /// Source of the original data
    static const QString dataSource;
};

}
