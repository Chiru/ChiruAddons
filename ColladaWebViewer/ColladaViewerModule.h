#pragma once

#include "WebSocketManager.h"
#include "IModule.h"
#include "StableHeaders.h"
#include "AssetAPI.h"
#include "AssetModule.h"
#include "IAssetUploadTransfer.h"

#include <QObject>


namespace ColladaViewer
{

class ColladaViewerModule : public IModule
{
    Q_OBJECT

public:
    ColladaViewerModule();
    ~ColladaViewerModule();

    /// IModule override.
    void Load();

    /// IModule override.
    void Initialize();

    /// IModule override.
    void Uninitialize();

    /// IModule override.
    void Update(f64 frametime);

    /// Utility functions

    // Loads a collada file to a string
    int loadColladaToString(string path, string &data);

public slots:

    // Sends a collada file to an asset server
    void sendColladaToServer(QString path);

    // Starts a server side process
    void serverProcess();

    // Starts a client side process
    void clientProcess();

    // Processes events and data that came from websocket manager
    void processEvent(QString event, QString data, QString clientId);

    void transferCompleted(IAssetUploadTransfer *transfer);
    void transferFailed(IAssetUploadTransfer *transfer);

    void remoteAssetChanged(QString localName, QString diskSource, IAssetStorage::ChangeType change);
    void storageAdded(AssetStoragePtr storage);


private:
    /// Pointer to WebSocketManager object
    WebSocketManager *websocketManager_;

    /// Pointer to assetAPI
    AssetAPI *assetAPI_;

    /// Pointer to assetModule
    AssetModule *assetModule_;

    /// Pointer to collada storage
    AssetStoragePtr storage_;

    bool isServer;
    unsigned short websocketPort;

    string remoteStorageUrl;
    string localStorageUrl;

    QStringList remoteAssets;

};

}
