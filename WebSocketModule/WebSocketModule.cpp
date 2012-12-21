#include "WebSocketModule.h"
#include "CoreDefines.h"
#include "TundraLogicModule.h"
#include "Server.h"
#include "Client.h"
#include "Framework.h"
#include "WSSyncManager.h"

#include "SceneImporter.h"

#include "SceneAPI.h"
#include "Scene.h"
#include "KristalliProtocolModule.h"
#include <kNet.h>

namespace WebSocketSync
{

WebSocketModule::WebSocketModule():
    IModule("WebSocket")
{


}

WebSocketModule::~WebSocketModule()
{
    SAFE_DELETE(syncmanager_);
    SAFE_DELETE(websocketmanager_);
}

void WebSocketModule::Load()
{
    framework_->RegisterDynamicObject("WebSocket", this);
}


void WebSocketModule::Initialize()
{
    // Getting needed pointers
    tundra_ = framework_->GetModule<TundraLogic::TundraLogicModule>();
    server_ = tundra_->GetServer().get();


    /// Initializing signal listeners

    bool check = connect(framework_->Scene(), SIGNAL(SceneAdded(QString)),
                         this, SLOT(registerSyncManager(QString)));
    Q_ASSERT(check);
    check = connect(framework_->Scene(), SIGNAL(SceneRemoved(QString)),
                    this, SLOT(removeSyncManager(QString)));
    Q_ASSERT(check);

    // Starts the main process when server is ready
    check = connect(server_, SIGNAL(ServerStarted()),
                         this, SLOT(mainProcess()), Qt::QueuedConnection);
    Q_ASSERT(check);


}


void WebSocketModule::Uninitialize()
{

}

void WebSocketModule::Update(f64 frametime)
{
        syncmanager_->Update(frametime);
}

void WebSocketModule::registerSyncManager(const QString name) {
    // Do not create syncmanager for dummy TundraServer scene.
    if (name == "TundraServer")
        return;


    // If scene is real deal, create websocketManager and syncManager

    // Initializing the websocket port
    if(framework_->HasCommandLineParameter("--wsport"))
    {
        QStringList params = framework_->CommandLineParameters("--wsport");
        if(params.length() > 0)
            websocketPort = params.at(0).toUShort();
    }else{
        websocketPort = 9002;
    }


    // Starting websocket server
    websocketmanager_ = new WebSocketManager(framework_, websocketPort);
    websocketmanager_->startServer();


    syncmanager_ = new WSSyncManager(tundra_, websocketmanager_);
    ScenePtr scene = framework_->Scene()->GetScene(name);
    syncmanager_->RegisterToScene(scene);
}

void WebSocketModule::removeSyncManager(const QString name){
    //--
}

void WebSocketModule::mainProcess()
{



}





} // End of namespace


extern "C"
{
DLLEXPORT void TundraPluginMain(Framework *fw)
{
    Framework::SetInstance(fw); // Inside this DLL, remember the pointer to the global framework object.
    IModule *module = new WebSocketSync::WebSocketModule();
    fw->RegisterModule(module);
}
}
