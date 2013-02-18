#pragma once

#include "WebSocketManager.h"
#include "IModule.h"
#include "StableHeaders.h"
#include "WSSyncManager.h"

#include <kNet.h>
#include <QObject>


namespace WebSocketSync
{

using namespace std;

class WebSocketModule : public IModule
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
