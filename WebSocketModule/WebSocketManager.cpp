#include "WebSocketManager.h"
#include "CoreStringUtils.h"
#include <boost/version.hpp>

namespace WebSocketSync
{

WebSocketManager::WebSocketManager(Framework* framework, unsigned short port) :

    framework_(framework),
    port(port)

{

}

WebSocketManager::~WebSocketManager()
{
    stopServer();
}


/// Socket events

void WebSocketManager::on_validate(connection_ptr con)
{
    LogInfo("Request for resource: " + con->get_resource());
    //cout << "origin: " << con->get_origin() << endl;

}

void WebSocketManager::on_open(connection_ptr con)
{

    u8 clientId = AllocateNewConnectionID();

    LogDebug("Web Client " + ToString((int)clientId) + " connected.");

    //Adding the new connection to the list of connections
    connections.insert(pair<u8, connection_ptr>(clientId, con));
    LogDebug("Web clients connected: " + QString::number(connections.size()));

    //Emitting "connected" event signal
    emit gotEvent("connected", "", clientId);

}

void WebSocketManager::on_close(connection_ptr con)
{

    LogDebug("Web Client disconnected.");

    const map<u8, connection_ptr>::const_iterator it = std::find_if(
       connections.begin(), connections.end(), boost::bind(&map<u8, connection_ptr>::value_type::second, _1) == con
    );


    // Emitting disconnected event
    emit gotEvent("disconnected", "", it->first);

    removeConnection(con);

}

void WebSocketManager::on_fail(connection_ptr con)
{
    LogDebug("Connection to Web Client failed.");

    const map<u8, connection_ptr>::const_iterator it = std::find_if(
       connections.begin(), connections.end(), boost::bind(&map<u8, connection_ptr>::value_type::second, _1) == con
   );

    // Emitting disconnected event
    emit gotEvent("disconnected", "", it->first);

    removeConnection(con);

}

void WebSocketManager::on_message(connection_ptr con, message_ptr msg)
{
    //LogDebug("Got message: " + msg->get_payload());

    Json::Value root;

    //Parses the message as JSON
    root = parseJson(msg->get_payload());

    if(root != 0)
    {
        //LogDebug(root.toStyledString());
        Json::Value event = root.get("event", "empty");
        Json::Value data = root.get("data", "empty");

        if(event == "empty")
            return;

        if(data == "empty")
            return;

        // Making sure that the event name is a string, so we don't get an exception
        if(!event.isString())
            return;

        QString eventName = QString::fromStdString(event.asString());


        const map<u8, connection_ptr>::const_iterator it = std::find_if(
           connections.begin(), connections.end(), boost::bind(&map<u8, connection_ptr>::value_type::second, _1) == con
        );

        if(it != connections.end()){
           emit gotEvent(eventName, data, it->first);
        }
    }
    else
        LogDebug("parseJson apparently failed");

}


/// Utility functions

u8 WebSocketManager::AllocateNewConnectionID() const
{
    u8 newID = 1;
    for(map<u8, server::connection_ptr>::const_iterator iter = connections.begin(); iter != connections.end(); ++iter)
        newID = std::max((int)newID, (int)(iter->first+1));

    return newID;
}

map<u8, server::connection_ptr>::iterator WebSocketManager::findClient(u8 clientId)
{

    map<u8, server::connection_ptr>::iterator it = connections.find(clientId);
    return it;
    /*
    const map<u8, connection_ptr>::iterator it = std::find_if(
       m.begin(), m.end(), boost::bind(&my_map::value_type::second, _1) == con
   );
   */
}

void WebSocketManager::removeConnection(connection_ptr con)
{


    //map<u8, connection_ptr>::iterator it = connections.find(getConId(con));
    map<u8, connection_ptr>::iterator it = std::find_if(
       connections.begin(), connections.end(), boost::bind(&map<u8, connection_ptr>::value_type::second, _1) == con
   );

    if (it == connections.end()) {
        /* This client has already disconnected, we can ignore this.
        This happens when there is a deliberate "soft" disconnection
        preceeding the "hard" socket read fail or disconnect ack message.
        return;*/
    }

    connections.erase(it);

    LogDebug("Web clients connected: " + QString::number(connections.size()));
}

void WebSocketManager::closeConnections()
{
    // Closing all connections
    if(connections.size() > 0) {
        for (map<u8, server::connection_ptr>::const_iterator iter = connections.begin();
             iter != connections.end(); ++iter )
        {

            iter->second->close(websocketpp::close::status::GOING_AWAY);

        }
    }
}

void WebSocketManager::cleanConnections()
{
    boost::posix_time::seconds sleepTime(320);

    while(true)
    {

        if(connections.size() > 0) {
            LogDebug("Checking for dead WebSockets...");

            for (map<u8, server::connection_ptr>::const_iterator iter = connections.begin();
                 iter != connections.end(); ++iter )
            {
                // Sending ping to the web clients. The connection should terminate if there is no response.
                iter->second->ping("ping");
                //cout << iter->first << ", state: " << iter->second->get_state() << endl;
            }
        }

        boost::this_thread::sleep(sleepTime);
    }
}

Json::Value WebSocketManager::parseJson(string s)
{
    // Let's parse it
    Json::Value root;
    Json::Reader reader;

    bool parsedSuccess = reader.parse(s, root, false);

    if(!parsedSuccess)
    {
        cout<<"Failed to parse JSON"<<endl <<reader.getFormatedErrorMessages() <<endl;
        return 0;
    }
    else
    {
        return root;
    }
}

string WebSocketManager::createEventMsg(string event, Json::Value &data)
{
    Json::Value json;
    json["data"] = data;
    json["event"] = event;
/*
    if(data.size() > 1)
        json.put_child("data", data);
    else
        json.add("data", data.front().second.data());

    stringstream buffer;
    write_json(buffer, json);
*/
    return json.toStyledString();
}


void WebSocketManager::sendJsonToClient(string jsonString, u8 clientId)
{
    map<u8, server::connection_ptr>::iterator it = findClient(clientId);
    it->second->send(jsonString);
}

void WebSocketManager::sendJsonToClients(string jsonString)
{
    for ( std::map<u8, server::connection_ptr>::const_iterator iter = connections.begin();
          iter != connections.end(); ++iter )
        iter->second->send(jsonString);

}

void WebSocketManager::startServer()
{
    LogDebug("Boost version used: " + QString::fromLocal8Bit(BOOST_LIB_VERSION));
    LogInfo("Starting WebSocket server on port " + QString::number(port));

    server::handler::ptr h(this);
    endpoint_ = new server(h);

    try {

        using namespace websocketpp::log;
        endpoint_->alog().unset_level(alevel::ALL);
        endpoint_->elog().unset_level(elevel::ALL);

        endpoint_->alog().set_level(alevel::CONNECT);
        endpoint_->alog().set_level(alevel::DISCONNECT);

        endpoint_->elog().set_level(elevel::RERROR);
        endpoint_->elog().set_level(elevel::FATAL);

        //Getting pointer to the right server listen function
        void(websocketpp::role::server<websocketpp::server>::*f)(uint16_t,size_t) =
                &websocketpp::role::server<websocketpp::server>::listen;

        LogDebug("Starting WSServer thread...");
        boost::shared_ptr<boost::thread> serverThread(new boost::thread(f, endpoint_, port, 1));
        listener = serverThread;

        LogDebug("Starting WS cleaning thread...");
        boost::shared_ptr<boost::thread> cleanerThread(new boost::thread(&WebSocketManager::cleanConnections, this));
        cleaner = cleanerThread;


    } catch (exception& e) {
        cerr << "WebSocketManager Exception: " << e.what() << endl;
    }

}

void WebSocketManager::stopServer()
{
    LogInfo("Stopping WebSocket server on port " + QString::number(port));

    try {

        // Closing all connections cleanly
        this->closeConnections();

        endpoint_->stop();

        if(listener->joinable()){
            listener->join();
            listener.reset();
            LogDebug("Removed WS listener thread");
        }

        cleaner->interrupt();

        if(cleaner->joinable()){
            cleaner->join();
            cleaner.reset();
            LogDebug("Removed WS cleaner thread");
        }


    } catch (exception& e) {
        cerr << "WebSocketManager Exception: " << e.what() << endl;
    }

}

} //End of namespace WebSocketSynch




