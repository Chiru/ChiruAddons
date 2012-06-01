// RdfVocabulary.js - RDF vocabulary that is used both internally and when communicating with preprocessor. */

// RDF vocabulary that is used both internally and when communicating with preprocessor.
var RdfVocabulary =
{
    // Base uri
    baseUri : "http://cie/",
    // namespace prefix
    namespacePrefix : "cie",
    // The preprocessor name
    sourceApplication : "source-application",
    // Geolocation, a 3D position vector or a name of a place
    geoLocation : "geo",
    // The date of the source data and preprocessing
    dateTime : "datetime",
    // Preprocessed data
    data : "data",
    // Metadata for the preprocessed data
    metadata : "metadata",
    // Source of the original data
    dataSource : "data-source",
};
RdfVocabulary.geoLocation = RdfVocabulary.baseUri + RdfVocabulary.geoLocation;
RdfVocabulary.dateTime = RdfVocabulary.baseUri + RdfVocabulary.dateTime;
RdfVocabulary.data = RdfVocabulary.baseUri + RdfVocabulary.data;
RdfVocabulary.metadata = RdfVocabulary.baseUri + RdfVocabulary.metadata;
RdfVocabulary.dataSource = RdfVocabulary.baseUri + RdfVocabulary.dataSource;
