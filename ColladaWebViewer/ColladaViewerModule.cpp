#include "ColladaViewerModule.h"
#include "CoreDefines.h"
#include "TundraLogicModule.h"
#include "Server.h"
#include "Client.h"

namespace ColladaViewer
{

ColladaViewerModule::ColladaViewerModule():
    IModule("ColladaViewer")
{
    websocketPort = 9002;
    colladaStoragePath = "../src/ChiruAddons/Scenes/ColladaStorage/";

}

ColladaViewerModule::~ColladaViewerModule()
{
    SAFE_DELETE(websocketManager_);
}

void ColladaViewerModule::Load()
{
    framework_->RegisterDynamicObject("ColladaViewer", this);
}


void ColladaViewerModule::Initialize()
{
    TundraLogic::TundraLogicModule* tundra_ = framework_->GetModule<TundraLogic::TundraLogicModule>();
    assetAPI_ = framework_->Asset();
    assetModule_ = framework_->GetModule<AssetModule>();

    //Initializing signal listeners
    bool check = connect(tundra_->GetServer().get(), SIGNAL(ServerStarted()),
                         this, SLOT(serverProcess()), Qt::QueuedConnection);
    Q_ASSERT(check);
    check = connect(tundra_->GetClient().get(), SIGNAL(Connected(UserConnectedResponseData *)),
                    this, SLOT(clientProcess()), Qt::QueuedConnection);
    Q_ASSERT(check);
}

void ColladaViewerModule::Uninitialize()
{

}

void ColladaViewerModule::Update(f64 frametime)
{

}


void ColladaViewerModule::serverProcess()
{
    LogInfo("Colladaviewer: I am a server!");
    websocketManager_ = new WebSocketManager(websocketPort);
    websocketManager_->startServer();

    //Listening for event signals from websocket manager
    bool check = connect(websocketManager_, SIGNAL(gotEvent(QString, QString, QString)),
                         SLOT(processEvent(QString, QString, QString)), Qt::QueuedConnection);
    Q_ASSERT(check);

    stringstream storageString;
    storageString << "type=LocalAssetStorage;name=colladaFiles;src=../src/ChiruAddons/Scenes/ColladaStorage/;trusted=true;default;";

    assetModule_->AddAssetStorage(QString::fromStdString(storageString.str()));
    storage_ = assetAPI_->GetAssetStorageByName(QString("colladaFiles"));
}

void ColladaViewerModule::clientProcess()
{
    LogInfo("Colladaviewer: I am a client!");

    stringstream storageString;
    storageString << "type=LocalAssetStorage;name=colladaFiles;src=http://127.0.0.1;default;";

    assetModule_->AddAssetStorage(QString::fromStdString(storageString.str()));
    storage_ = assetAPI_->GetAssetStorageByName(QString("colladaFiles"));
    cout << storage_ ->Name().toStdString() <<endl;

    //Listening collada export signal from ObjectCapture module

    /*
    //WORKS!
    bool check = QObject::connect(framework_->GetModuleByName(QString("ObjectCapture")),
                                  SIGNAL(colladaExportFinalized(QString)), this,
                    SLOT(sendColladaToServer(QString)), Qt::QueuedConnection);
                    */

    //Just for testing without ObjectCapture module
    sendColladaToServer(QString("../src/CapturedHorse.dae"));

    //Q_ASSERT(check);

}

void ColladaViewerModule::processEvent(QString event, QString data, QString clientId)
{
    if (event == "requestCollada"){
        stringstream path;
        path << colladaStoragePath;
        path << data.toStdString(); // <- Adding the collada file name

        string collada;

        if(loadColladaToString(path.str(), collada) != -1)
        {
            cout << "Req. file found: " << path.str() <<endl;
            string json;

            json = websocketManager_->createEventMsg("loadCollada", collada);
            cout << json.substr(10, 100) << endl;
            websocketManager_->sendJsonToClient(json, clientId.toStdString());
        }
        else{
            return;
        }
    }

}


int ColladaViewerModule::loadColladaToString(string path, string &data)
{
    ifstream file(path.c_str());
    stringstream buffer;

    cout << "Loading collada" <<endl;
    if (file){
        buffer << file.rdbuf();
        file.close();
        data = buffer.str();

        return 0;
    }

    return -1;
}

void ColladaViewerModule::sendColladaToServer(QString path)
{
    cout << "Local collada path: " << path.toStdString() << endl;



    assetAPI_->UploadAssetFromFile(path, storage_, "capturedObject.dae");
   //cout << "loaded file to vector" <<endl;

}

/*
int ColladaViewerModule::storeCollada(string destPath, string collada)
{
    /// TODO:
}
*/

} //End of namespace ColladaViewer


extern "C"
{
DLLEXPORT void TundraPluginMain(Framework *fw)
{
    Framework::SetInstance(fw); // Inside this DLL, remember the pointer to the global framework object.
    IModule *module = new ColladaViewer::ColladaViewerModule();
    fw->RegisterModule(module);
}
}
