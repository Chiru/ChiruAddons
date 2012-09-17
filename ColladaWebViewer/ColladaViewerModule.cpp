#include "ColladaViewerModule.h"
#include "CoreDefines.h"
#include "TundraLogicModule.h"
#include "Server.h"
#include "Client.h"
#include "Framework.h"

namespace ColladaViewer
{

ColladaViewerModule::ColladaViewerModule():
    IModule("ColladaViewer")
{
    remoteStorageUrl = "";

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

    // Listens if there are new added storages
    bool check = connect(assetAPI_, SIGNAL(AssetStorageAdded(AssetStoragePtr)), this, SLOT(storageAdded(AssetStoragePtr)),
                                    Qt::QueuedConnection);
    Q_ASSERT(check);

    // Setting up the remote storage
    if(remoteStorageUrl != ""){
        stringstream storageString;
        storageString << "type=HttpAssetStorage;name=colladaStorage;src=" <<
                         remoteStorageUrl << ";trusted=true;autodiscoverable=false;replicated=false;readonly=false;default=false;";

        assetAPI_->DeserializeAssetStorageFromString(QString::fromStdString(storageString.str()), true);

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
    check = connect(websocketManager_, SIGNAL(gotEvent(QString, QString, QString)),
                         SLOT(processEvent(QString, QString, QString)), Qt::QueuedConnection);
    Q_ASSERT(check);

    // Listening for storage refresh requests from tundra client
    check = connect(server_, SIGNAL(MessageReceived(UserConnection*,kNet::packet_id_t,kNet::message_id_t,const char*,size_t)),
                    SLOT(parseKnetMessage(UserConnection*,kNet::packet_id_t,kNet::message_id_t,const char*,size_t)));

    Q_ASSERT(check);

}

void ColladaViewerModule::processEvent(QString event, QString data, QString clientId)
{
    if(event.isEmpty() || clientId.isEmpty())
        return;

    // Sends information related to remote storage to the connected web client
    if (event == "connected"){

        //Sending a list of collada files stored in storage
        if(!assetList.isEmpty()){
            string json;

            ptree data;
            data.put<string>("list", assetList.join(", ").toStdString());
            data.put<string>("storageUrl", remoteStorageUrl);

            json = websocketManager_->createEventMsg("colladaList", data);
            websocketManager_->sendJsonToClient(json, clientId.toStdString());
        }
    }
}

void ColladaViewerModule::parseKnetMessage(UserConnection *connection, kNet::packet_id_t, kNet::message_id_t id, const char* data, size_t numBytes)
{
    if (id == 200){
        LogDebug("Got a remote collada storage refresh request");
        if(remoteStorage_)
           remoteStorage_->RefreshAssetRefs();
    }
}

void ColladaViewerModule::clientProcess()
{
    isServer = false;

    // Listening collada export signal from ObjectCapture module
    bool check = QObject::connect(framework_->GetModuleByName(QString("ObjectCapture")),
                                  SIGNAL(assetUploaded(QString)), this,
                    SLOT(uploadCompleted(QString)), Qt::QueuedConnection);

    Q_ASSERT(check);

}


void ColladaViewerModule::storageAdded(AssetStoragePtr storage)
{
    if(storage->Type() == "HttpAssetStorage" && storage->Name() == "colladaStorage"){
        remoteStorage_ = storage;
        LogInfo("Added a remote collada storage: " + remoteStorage_->ToString());

        // Listening for assets changes in the remote storage
        bool check = connect(remoteStorage_.get(), SIGNAL(AssetChanged(QString,QString,IAssetStorage::ChangeType)),
                             this, SLOT(remoteAssetChanged(QString,QString,IAssetStorage::ChangeType)), Qt::QueuedConnection);
        Q_ASSERT(check);


        //Getting all the asset refs from remote storage
        remoteStorage_->RefreshAssetRefs();

    }
}


void ColladaViewerModule::remoteAssetChanged(QString localName, QString diskSource, IAssetStorage::ChangeType change)
{
    switch(change)
    {
    case 0: {
        LogInfo("A new file was found at: " + remoteStorage_->ToString() + " " + localName);

        if(websocketManager_){

            //Adding the new collada file name to asset list
            if (!assetList.contains(localName))
                assetList.append(localName);

            // Tundra server informs the web browser clients about the new captured collada file loaded in the local storage
            // but does not send the file instantly because that might not be desired by the web client user

            string json;

            ptree data;
            data.put<string>("fileName", localName.toStdString());

            json = websocketManager_->createEventMsg("newCollada", data);
            websocketManager_->sendJsonToClients(json);
        }


        break;
    }
    case 1:
        LogInfo("A file was modified at: " + remoteStorage_->ToString() + localName);
        break;
    case 2:
        LogInfo("A file was deleted from: " + remoteStorage_->ToString() + localName);

        if(websocketManager_){
            assetList.removeOne(localName);

            string json;

            ptree data;
            data.put<string>("fileName", localName.toStdString());

            json = websocketManager_->createEventMsg("removeCollada", data);
            websocketManager_->sendJsonToClients(json);
        }

        break;
    }
}

/// Transfer handler functions

void ColladaViewerModule::uploadCompleted(QString assetRef)
{

    if(!isServer)
    {
        LogInfo("Captured collada uploaded to: ");

        //Sending a storage refresh request to tundra server
        MsgRequestRefresh knetMsg;
        knetMsg.userID = client_->ConnectionId();

        // TUNDRA_MULTICONNECTION flag is in CMakeLists.txt at chiru root

#ifdef TUNDRA_MULTICONNECTION
        Ptr(kNet::MessageConnection) connection = client_->GetConnection(client_->getActiveScenename());
#else
        Ptr(kNet::MessageConnection) connection = client_->GetConnection();
#endif

        connection.ptr()->Send(knetMsg);
    }
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
