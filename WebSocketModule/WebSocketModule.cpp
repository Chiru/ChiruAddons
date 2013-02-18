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

    // Initializing the module only if we are running on a server
    // Checking commandline parameters for "--server" is a temporary way of checking if we are a server until a better solution is found
    // The problem here is that SceneAdded signals are emitted before ServerStarted signal
    /// TODO: Fix a crash that occurs when native tundra client that runs a websocketmodule, tries to connect to a tundra server that also runs a websocket module
    /// on a same computer. This crash is avoided by running the tundra client e.g. with --config viewer-browser.xml which has no websocket module

    if(framework_->HasCommandLineParameter("--server")){
        LogDebug("WebSocketModule: I am a server!");

        // Getting needed pointers
        tundra_ = framework_->GetModule<TundraLogic::TundraLogicModule>();
        server_ = tundra_->GetServer().get();

        // Listening scene added/remove signals and registering the WSSyncManager
        bool check = connect(framework_->Scene(), SIGNAL(SceneAdded(QString)),
                             this, SLOT(registerSyncManager(QString)));
        Q_ASSERT(check);

        check = connect(framework_->Scene(), SIGNAL(SceneRemoved(QString)),
                        this, SLOT(removeSyncManager(QString)));
        Q_ASSERT(check);
    }else{
        LogDebug("WebSocketModule: I am a client!");
    }

}


void WebSocketModule::Uninitialize()
{

}

void WebSocketModule::Update(f64 frametime)
{
    if(syncmanager_){
        syncmanager_->Update(frametime);
    }
}


void WebSocketModule::registerSyncManager(const QString name) {
    // Do not create syncmanager for dummy TundraServer scene.
    if (name == "TundraServer")
        return;

    // If scene is a real deal, create websocketManager and syncManager

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
