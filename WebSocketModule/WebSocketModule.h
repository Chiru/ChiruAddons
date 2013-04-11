#pragma once

#include "WebSocketManager.h"
#include "IModule.h"
#include "StableHeaders.h"
#include "WSSyncManager.h"

#include <kNet.h>
#include <QObject>
#include <boost/enable_shared_from_this.hpp>

namespace WebSocketSync
{

using namespace std;

class WebSocketModule : public IModule, public boost::enable_shared_from_this<WebSocketModule>
{
    Q_OBJECT

public:
    WebSocketModule();
    ~WebSocketModule();

    /// IModule override.
    void Load();

    /// IModule override.
    void Initialize();

    /// IModule override.
    void Uninitialize();

    /// IModule override.
    void Update(f64 frametime);

    WSSyncManager* GetSyncManager();


public slots:


private slots:
    void registerSyncManager(const QString);
    void removeSyncManager(const QString);

private:
    /// Pointer to WebSocketManager object
    WebSocketManager *websocketmanager_;
    WSSyncManager *syncmanager_;
    TundraLogic::TundraLogicModule *tundra_;


    TundraLogic::Server *server_;

    unsigned short websocketPort;


};

}
