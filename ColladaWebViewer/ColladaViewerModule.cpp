#include "ColladaViewerModule.h"
#include "CoreDefines.h"
#include "TundraLogicModule.h"
#include "Server.h"
#include "Client.h"

#include <QFileInfo>

namespace ColladaViewer
{

ColladaViewerModule::ColladaViewerModule():
    IModule("ColladaViewer")
{

    remoteStorageUrl = "";
    localStorageUrl = "../src/ChiruAddons/Scenes/ColladaStorage/";

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
    // Getting needed pointers
    TundraLogic::TundraLogicModule* tundra_ = framework_->GetModule<TundraLogic::TundraLogicModule>();
    assetAPI_ = framework_->Asset();
    client_ = tundra_->GetClient().get();
    server_ = tundra_->GetServer().get();


    // Getting remote collada storage url
    if(framework_->HasCommandLineParameter("--remoteColladaStorage"))
    {
        QStringList params = framework_->CommandLineParameters("--remoteColladaStorage");
        if(params.length() > 0)
            remoteStorageUrl = params.at(0).toStdString();
    }


    /// Initializing signal listeners

    // Starts the server process if the module runs on tundra server
    bool check = connect(server_, SIGNAL(ServerStarted()),
                         this, SLOT(serverProcess()), Qt::QueuedConnection);
    Q_ASSERT(check);

    // Starts the client process if the module runs on tundra client
    check = connect(client_, SIGNAL(Connected(UserConnectedResponseData *)),
                    this, SLOT(clientProcess()), Qt::QueuedConnection);
    Q_ASSERT(check);

    // Listens if there are new added storages
    check = connect(assetAPI_, SIGNAL(AssetStorageAdded(AssetStoragePtr)), this, SLOT(storageAdded(AssetStoragePtr)),
                                    Qt::QueuedConnection);
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
    isServer = true;

    // Setting up the remote storage
    if(remoteStorageUrl != ""){
        stringstream storageString;
        storageString << "type=HttpAssetStorage;name=colladaStorage;src=" <<
                         remoteStorageUrl << ";trusted=true;readonly=false;default=false;";

        assetAPI_->DeserializeAssetStorageFromString(QString::fromStdString(storageString.str()), true);

    }

    // Setting up the local storage
    if(localStorageUrl != ""){
        stringstream storageString;
        storageString << "type=LocalAssetStorage;name=localColladaStorage;src=" <<
                         localStorageUrl << ";trusted=true;autodiscoverable=true;readonly=false;default=false;";

        assetAPI_->DeserializeAssetStorageFromString(QString::fromStdString(storageString.str()), false);

    }

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
    websocketManager_ = new WebSocketManager(websocketPort);
    websocketManager_->startServer();

    // Listening for event signals from websocket manager
    bool check = connect(websocketManager_, SIGNAL(gotEvent(QString, QString, QString)),
                         SLOT(processEvent(QString, QString, QString)), Qt::QueuedConnection);
    Q_ASSERT(check);

    // Listening for storage refresh requests from tundra client
    check = connect(server_, SIGNAL(MessageReceived(UserConnection*,kNet::packet_id_t,kNet::message_id_t,const char*,size_t)),
                    SLOT(parseKnetMessage(UserConnection*,kNet::packet_id_t,kNet::message_id_t,const char*,size_t)));

    Q_ASSERT(check);

}

void ColladaViewerModule::parseKnetMessage(UserConnection *connection, kNet::packet_id_t, kNet::message_id_t id, const char* data, size_t numBytes)
{
    if (id == 200){
        LogDebug("Got a remote collada storage refresh request");
        if(storage_)
           storage_->RefreshAssetRefs();
    }
}

void ColladaViewerModule::clientProcess()
{
    isServer = false;

    // Setting up the storage
    stringstream storageString;
    if(remoteStorageUrl != ""){
        storageString << "type=HttpAssetStorage;name=colladaStorage;src=" <<
                         remoteStorageUrl << ";trusted=true;readonly=false;default=false;";

        assetAPI_->DeserializeAssetStorageFromString(QString::fromStdString(storageString.str()), true);

    }

    // Listening collada export signal from ObjectCapture module
    bool check = QObject::connect(framework_->GetModuleByName(QString("ObjectCapture")),
                                  SIGNAL(colladaExportFinalized(QString)), this,
                    SLOT(sendFileToRemoteStorage(QString)), Qt::QueuedConnection);

    Q_ASSERT(check);

    // Just for testing uploading a captured object without ObjectCapture module
    //sendFileToRemoteStorage(QString("../src/ChiruAddons/Scenes/ColladaStorage/capturedObject.dae"));

}

void ColladaViewerModule::processEvent(QString event, QString data, QString clientId)
{
    // Web browser client requests for a collada file
    if (event == "requestCollada"){
        stringstream path;

        /// The requested collada is loaded from local storage

        // Making sure that no path was added with the file name to prevent web client users
        // from requesting files from system folders. e.g request "/path/to/folder/CapturedObject.dae" is
        // changed just to "CapturedObject.dae" and the system just searches for it from the local collada storage

        QFileInfo fileInfo(data);
        QString fileName = fileInfo.fileName();
        LogDebug("Parsed filename: " + fileName);
        path << localStorageUrl;
        path << fileName.toStdString();

        string collada;

        // Loads a collada file from a local storage and sends it to webclient who requested it
        if(loadColladaToString(path.str(), collada) != -1)
        {
            LogDebug("Req. file found: " + path.str());
            string json;

            json = websocketManager_->createEventMsg("loadCollada", collada);
            cout << json.substr(10, 100) << endl;
            websocketManager_->sendJsonToClient(json, clientId.toStdString());
        }else{
            LogError("ColladaViewer: Requested file not found: " + path.str());
        }
    }
}

void ColladaViewerModule::storageAdded(AssetStoragePtr storage)
{
    if(storage->Type() == "HttpAssetStorage" && storage->Name() == "colladaStorage"){
        storage_ = storage;
        LogInfo("Added a remote collada storage: " + storage_->ToString());

        // Listening for assets changes in the remote storage
        bool check = connect(storage_.get(), SIGNAL(AssetChanged(QString,QString,IAssetStorage::ChangeType)),
                             this, SLOT(remoteAssetChanged(QString,QString,IAssetStorage::ChangeType)), Qt::QueuedConnection);
        Q_ASSERT(check);


        //Getting all the asset refs from remote storage
        storage_->RefreshAssetRefs();

    }else if(storage->Type() == "LocalAssetStorage" && storage->Name() == "localColladaStorage"){
        localStorage_ = storage;
        LogInfo("Added a local collada storage: " + localStorage_->ToString());

        // Listening for assets changes in the local storage
        bool check = connect(localStorage_.get(), SIGNAL(AssetChanged(QString,QString,IAssetStorage::ChangeType)),
                             this, SLOT(localAssetChanged(QString,QString,IAssetStorage::ChangeType)), Qt::QueuedConnection);
        Q_ASSERT(check);
    }
}


void ColladaViewerModule::remoteAssetChanged(QString localName, QString diskSource, IAssetStorage::ChangeType change)
{
    switch(change)
    {
    case 0: {
        LogInfo("A new file was found at: " + storage_->ToString() + " " + localName);

        if(isServer){
            LogDebug("Requesting asset from the remote storage...");
            QString fullUrl = storage_->GetFullAssetURL(localName);

            //Requesting the collada file for download in a hackyish way
            AssetTransferPtr con = assetAPI_->GetProviderForAssetRef(fullUrl, "Binary")->RequestAsset(fullUrl, "Binary");

            // This call tries to download the collada files as ogre meshes even when forcing the type to binary
            // and crashes :|
            //AssetTransferPtr con = assetAPI_->RequestAsset(storage_->GetFullAssetURL(localName), "Binary");

            if(con)
            {
                bool check = connect(con.get(), SIGNAL(Downloaded(IAssetTransfer *)), this,
                                     SLOT(downloadCompleted(IAssetTransfer *)), Qt::QueuedConnection);
                Q_ASSERT(check);
                check = connect(con.get(), SIGNAL(Failed(IAssetTransfer *, QString)), this,
                                SLOT(downloadFailed(IAssetTransfer *, QString)), Qt::QueuedConnection);
                Q_ASSERT(check);
            }
        }

        break;
    }
    case 1:
        LogInfo("A file was modified at: " + storage_->ToString() + localName);
        break;
    case 2:
        LogInfo("A file was deleted from: " + storage_->ToString() + localName);
        break;
    }
}


void ColladaViewerModule::localAssetChanged(QString localName, QString diskSource, IAssetStorage::ChangeType change)
{
    switch(change)
    {
    case 0: {
        LogInfo("A new file was added to: " + localStorage_->ToString() + " " + localName);

        if(isServer){
            // Tundra server informs the web browser clients about the new captured collada file loaded in the local storage
            // but does not send the file instantly because that might not be desired by the web client user

            string json;
            json = websocketManager_->createEventMsg("newCollada", localName.toStdString());
            websocketManager_->sendJsonToClients(json);
        }
        break;
    }
    case 1:
        LogInfo("A file was modified at: " + localStorage_->ToString() + localName);
        break;
    case 2:
        LogInfo("A file was deleted from: " + localStorage_->ToString() + localName);
        break;
    }
}


int ColladaViewerModule::loadColladaToString(string path, string &data)
{
    ifstream file(path.c_str());
    stringstream buffer;

    LogDebug("Loading collada..." + QString::fromStdString(path));

    if (file){
        buffer << file.rdbuf();
        file.close();
        data = buffer.str();

        return 0;
    }

    return -1;
}


void ColladaViewerModule::sendFileToRemoteStorage(QString path)
{
    if(path.isEmpty())
        return;

    if(assetAPI_->GetAssetStorageByName("colladaStorage"))
    {
        LogDebug("Captured collada was saved temporarily to: " + path);

        QFileInfo pathInfo(path);
        AssetUploadTransferPtr con =  assetAPI_->UploadAssetFromFile(path,
                                         QString("colladaStorage"), pathInfo.fileName());

        if(con)
        {
            bool check = connect(assetAPI_, SIGNAL(AssetUploaded(QString)),
                                 SLOT(uploadCompleted(QString)), Qt::QueuedConnection);
            Q_ASSERT(check);
            check = connect(con.get(), SIGNAL(Failed(IAssetUploadTransfer *)),
                            SLOT(uploadFailed(IAssetUploadTransfer *)), Qt::QueuedConnection);
            Q_ASSERT(check);

            LogInfo("Uploading exported collada file to a remote storage...");
        }
    }

}


void ColladaViewerModule::sendFileToLocalStorage(QString fileRef)
{
    if(fileRef.isEmpty())
        return;

    if(assetAPI_->GetAssetStorageByName("localColladaStorage"))
    {
        LogDebug("Local collada path: " + fileRef);

        QFileInfo pathInfo(assetAPI_->DesanitateAssetRef(fileRef));
        AssetUploadTransferPtr con =  assetAPI_->UploadAssetFromFile(fileRef,
                                         QString("localColladaStorage"), pathInfo.fileName());

        if(con)
        {
            bool check = connect(assetAPI_, SIGNAL(AssetUploaded(QString)),
                                 SLOT(uploadCompleted(QString)), Qt::QueuedConnection);
            Q_ASSERT(check);
            check = connect(con.get(), SIGNAL(Failed(IAssetUploadTransfer *)),
                            SLOT(uploadFailed(IAssetUploadTransfer *)), Qt::QueuedConnection);
            Q_ASSERT(check);

            LogInfo("Saving collada file to a local storage...");
        }
    }
}


void ColladaViewerModule::uploadCompleted(QString assetRef)
{

    if(!isServer)
    {
        LogInfo("Captured collada uploaded to: ");

        //Sending a storage refresh request to tundra server
        MsgRequestRefresh knetMsg;
        knetMsg.userID = client_->ConnectionId();

        Ptr(kNet::MessageConnection) connection = client_->GetConnection(client_->getActiveScenename());

        connection.ptr()->Send(knetMsg);
    }
}

void ColladaViewerModule::uploadFailed(IAssetUploadTransfer *transfer)
{

    LogError("Error: Failed to upload file " + transfer->DestinationName());

}

void ColladaViewerModule::downloadCompleted(IAssetTransfer *transfer)
{
    LogInfo("Downloaded a collada-file: " +transfer->SourceUrl());
    if(localStorage_ && transfer){
        AssetPtr asset = transfer->Asset();
        if(asset && !asset->DiskSource().isEmpty())
            sendFileToLocalStorage(asset->DiskSource());
    }
}

void ColladaViewerModule::downloadFailed(IAssetTransfer *transfer, QString reason)
{

    LogError("Error: Failed to download a collada: " + reason);
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
