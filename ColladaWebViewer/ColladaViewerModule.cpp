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

    remoteStorageUrl = "http://chiru.cie.fi/colladaStorage";
    localStorageUrl = "..src/ChiruAddons/Scenes/ColladaStorage/";

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
    // Getting pointers to asset modules and Tundra logic module
    TundraLogic::TundraLogicModule* tundra_ = framework_->GetModule<TundraLogic::TundraLogicModule>();
    assetAPI_ = framework_->Asset();
    assetModule_ = framework_->GetModule<AssetModule>();

    /// Initializing a remote storage for uploading/downloading captured collada files

    // Getting remote collada storage url
    if(framework_->HasCommandLineParameter("--remoteColladaStorage"))
    {
        QStringList params = framework_->CommandLineParameters("--remoteColladaStorage");
        if(params.length() > 0)
            remoteStorageUrl = params.at(0).toStdString();
    }



    /// Initializing signal listeners

    // Starts the server process if the module runs on tundra server
    bool check = connect(tundra_->GetServer().get(), SIGNAL(ServerStarted()),
                         this, SLOT(serverProcess()), Qt::QueuedConnection);
    Q_ASSERT(check);

    // Starts the client process if the module runs on tundra client
    check = connect(tundra_->GetClient().get(), SIGNAL(Connected(UserConnectedResponseData *)),
                    this, SLOT(clientProcess()), Qt::QueuedConnection);
    Q_ASSERT(check);

    // Listens if there are new added storages
    check = connect(assetAPI_, SIGNAL(AssetStorageAdded(AssetStoragePtr)), this, SLOT(storageAdded(AssetStoragePtr)),
                                    Qt::QueuedConnection);
    Q_ASSERT(check);


    // Setting up the storage
    stringstream storageString;
    if(remoteStorageUrl != ""){
        storageString << "type=HttpAssetStorage;name=colladaStorage;src=" <<
                         remoteStorageUrl << ";trusted=true;autodiscoverable=true;liveupdate=true;liveupload=true;readonly=false;default=false;";

        assetAPI_->DeserializeAssetStorageFromString(QString::fromStdString(storageString.str()), true);

    }

}


void ColladaViewerModule::Uninitialize()
{

}

void ColladaViewerModule::Update(f64 frametime)
{


}

void ColladaViewerModule::storageAdded(AssetStoragePtr storage)
{
    if(storage->Type() == "HttpAssetStorage"){
        storage_ = storage;
        LogInfo("Created a remote collada storage: " + storage_->ToString());

        bool check = connect(storage_.get(), SIGNAL(AssetChanged(QString,QString,IAssetStorage::ChangeType)),
                             this, SLOT(remoteAssetChanged(QString,QString,IAssetStorage::ChangeType)), Qt::QueuedConnection);
        Q_ASSERT(check);

        //Getting all the asset refs in the remote storage
        storage_->RefreshAssetRefs();

    }
}

void ColladaViewerModule::remoteAssetChanged(QString localName, QString diskSource, IAssetStorage::ChangeType change)
{
    switch(change)
    {
    case 0:
        LogInfo("Remote collada storage added: " +localName);
        break;
    case 1:
        LogInfo("Remote collada storage modified: " +localName);
        break;
    case 2:
        LogInfo("Remote collada storage deleted: " +localName);
        break;
    }

}


void ColladaViewerModule::serverProcess()
{
    LogInfo("Colladaviewer: I am a server!");

    isServer = true;

    // Initializing websocket port
    if(framework_->HasCommandLineParameter("--wsport"))
    {
        QStringList params = framework_->CommandLineParameters("--wsport");
        if(params.length() > 0)
            websocketPort = params.at(0).toUShort();
    }else{
        websocketPort = 9002;
    }


    websocketManager_ = new WebSocketManager(websocketPort);
    websocketManager_->startServer();

    // Listening for event signals from websocket manager
    bool check = connect(websocketManager_, SIGNAL(gotEvent(QString, QString, QString)),
                         SLOT(processEvent(QString, QString, QString)), Qt::QueuedConnection);
    Q_ASSERT(check);

}


void ColladaViewerModule::clientProcess()
{
    LogInfo("Colladaviewer: I am a client!");

    isServer = false;

    // Listening collada export signal from ObjectCapture module

    /*
    bool check = QObject::connect(framework_->GetModuleByName(QString("ObjectCapture")),
                                  SIGNAL(colladaExportFinalized(QString)), this,
                    SLOT(sendColladaToServer(QString)), Qt::QueuedConnection);
                    */

    // Just for testing uploading a captured object without ObjectCapture module
    sendColladaToServer(QString("../src/ChiruAddons/Scenes/ColladaStorage/capturedObject.dae"));

    //Q_ASSERT(check);



}

void ColladaViewerModule::processEvent(QString event, QString data, QString clientId)
{
    if (event == "requestCollada"){
        stringstream path;
        path << "../src/ChiruAddons/Scenes/ColladaStorage/";
        path << data.toStdString(); // <- Adding the collada file name to the file path

        string collada;

        // Loads a collada file from a local storage and sends it to webclient who requested it
        if(loadColladaToString(path.str(), collada) != -1)
        {
            cout << "Req. file found: " << path.str() <<endl;
            string json;

            json = websocketManager_->createEventMsg("loadCollada", collada);
            cout << json.substr(10, 100) << endl;
            websocketManager_->sendJsonToClient(json, clientId.toStdString());
        }else{
            stringstream error;
            error << "ColladaViewer: Requested file not found: ";
            error << path.str();
            LogError(error.str());
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
    if(remoteStorageUrl != "")
    {
        cout << "Local collada path: " << path.toStdString() << endl;
        AssetUploadTransferPtr con =  assetAPI_->UploadAssetFromFile(path,
                                              QString("colladaStorage"), QString("capturedObject.dae"));

        if(con)
        {
            bool check = connect(con.get(), SIGNAL(Completed(IAssetUploadTransfer *)),
                                 SLOT(transferCompleted(IAssetUploadTransfer *)), Qt::QueuedConnection);
            Q_ASSERT(check);
            check = connect(con.get(), SIGNAL(Failed(IAssetUploadTransfer *)),
                            SLOT(transferFailed(IAssetUploadTransfer *)), Qt::QueuedConnection);
            Q_ASSERT(check);
        }

    }

}

void ColladaViewerModule::transferCompleted(IAssetUploadTransfer *transfer)
{
    stringstream msg;

    msg << "Captured collada uploaded to: " << storage_->ToString().toStdString();
    LogInfo(msg.str());


    // NOTE: Possible bug in tundra: Trying to point to transfer ptr methods here causes a segfault.
    // Maybe IAssetUploadTransfer object gets deleted too soon?
    /*
    stringstream msg;
    msg << "Upload of " << transfer->SourceFilename().toStdString() << " to "
    << transfer->DestinationName().toStdString() << " completed!";

    cout << msg.str() <<endl;
    */
}

void ColladaViewerModule::transferFailed(IAssetUploadTransfer *transfer)
{

    stringstream msg;

    msg << "Error: Failed to upload captured collada to: " << storage_->ToString().toStdString();
    LogError(msg.str());

    // NOTE: Possible bug in tundra: Trying to point to transfer ptr methods here causes a segfault.
    // Maybe IAssetUploadTransfer object gets deleted too soon?

    /*
    stringstream msg;
    msg << "Error: Upload of " << transfer->SourceFilename().toStdString() << " to "
    << transfer->DestinationName().toStdString() << " failed!";

    LogError(msg.str().c_str());
    */
}



} // End of namespace colladaviewer


extern "C"
{
DLLEXPORT void TundraPluginMain(Framework *fw)
{
    Framework::SetInstance(fw); // Inside this DLL, remember the pointer to the global framework object.
    IModule *module = new ColladaViewer::ColladaViewerModule();
    fw->RegisterModule(module);
}
}
