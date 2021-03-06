#pragma once

#include "WebSocketManager.h"
#include "IModule.h"
#include "StableHeaders.h"
#include "AssetAPI.h"
#include "AssetModule.h"
#include "HttpAssetProvider.h"

#include <kNet.h>
#include <QObject>


namespace ColladaViewer
{

using namespace std;

/// Network message for requesting collada storage refresh
struct MsgRequestRefresh
{
        MsgRequestRefresh()
        {
            InitToDefault();
        }

        MsgRequestRefresh(const char *data, size_t numBytes)
        {
            InitToDefault();
            kNet::DataDeserializer dd(data, numBytes);
            DeserializeFrom(dd);
        }

        void InitToDefault()
        {
            reliable = defaultReliable;
            inOrder = defaultInOrder;
            priority = defaultPriority;
        }

        enum { messageID = 200 };
        static inline const char * const Name() { return "RefreshRequest"; }

        static const bool defaultReliable = true;
        static const bool defaultInOrder = true;
        static const u32 defaultPriority = 100;

        bool reliable;
        bool inOrder;
        u32 priority;

        u8 userID;

        inline size_t Size() const
        {
            return 1;
        }

        inline void SerializeTo(kNet::DataSerializer &dst) const
        {
            dst.Add<u8>(userID);
        }

        inline void DeserializeFrom(kNet::DataDeserializer &src)
        {
            userID = src.Read<u8>();
        }

};

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

public slots:

    // Starts a server side process
    void serverProcess();

    // Starts a client side process
    void clientProcess();

    // Processes events and data that came from websocket manager
    void processEvent(QString event, QString data, QString clientId);

    void uploadCompleted(QString assetRef);


    /// Storage event handlers

    void remoteAssetChanged(QString localName, QString diskSource, IAssetStorage::ChangeType change);
    void storageAdded(AssetStoragePtr storage);

    void parseKnetMessage(UserConnection *connection, kNet::packet_id_t, kNet::message_id_t id, const char* data, size_t numBytes);


private:
    /// Pointer to WebSocketManager object
    WebSocketManager *websocketManager_;

    /// Pointer to assetAPI
    AssetAPI *assetAPI_;

    /// Pointers to collada storages
    AssetStoragePtr remoteStorage_;

    TundraLogic::Client *client_;
    TundraLogic::Server *server_;

    bool isServer;
    unsigned short websocketPort;

    string remoteStorageUrl;

    /// A List for keeping track of collada files that are stored in the remote storage
    // This list is send to web clients when they connect so they know what assets they can currently request
    QStringList assetList;

};

}
