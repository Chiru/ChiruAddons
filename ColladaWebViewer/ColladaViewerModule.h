#pragma once

#include "WebSocketManager.h"
#include "IModule.h"
#include "StableHeaders.h"
#include "AssetAPI.h"
#include "AssetModule.h"

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

    int loadColladaToString(string path, string &data);
    //void createSocket();
    //void connectToServer

public slots:
    void sendColladaToServer(QString path);
    void serverProcess();
    void clientProcess();
    void processEvent(QString event, QString data, QString clientId);


private:
    WebSocketManager *websocketManager_;
    AssetAPI *assetAPI_;
    AssetModule *assetModule_;
    AssetStoragePtr storage_;

    bool isServer;
    unsigned short websocketPort;
    string colladaStoragePath;

};

}
