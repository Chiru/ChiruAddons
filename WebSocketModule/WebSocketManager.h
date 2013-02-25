#pragma once
#include <websocketpp/websocketpp.hpp>
#include <boost/thread.hpp>
//Define this before including json_parser, if there is boost spirit grammar initialized on multiple threads
//#define BOOST_SPIRIT_THREADSAFE //requires boost threads
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp> //uses spirit headers
//#include <boost/foreach.hpp>
#include <string>
#include <cstring>
#include <sstream>
#include <set>
#include <exception>
#include <qstring.h>
#include <jsoncpp/json/json.h>

#include "StableHeaders.h"

class Framework;

namespace WebSocketSync
{

using websocketpp::server;
using boost::property_tree::ptree;

using namespace std;


class WebSocketManager : public QObject, public server::handler
{

    Q_OBJECT

public:
    WebSocketManager(Framework* framework, unsigned short port);
    ~WebSocketManager();

    // Starts the websocket server and event listeners
    void startServer();

    // Gracefully stops the server
    void stopServer();



    /// Utility functions

    // Creates a client id
    u8 AllocateNewConnectionID() const;

    // Finds a connected client by client id
    map<u8, server::connection_ptr>::iterator findClient(u8 clientId);

    // Parses a JSON string and creates a ptree from the JSON data
    Json::Value parseJson(string s);

    // Creates an event message and converts it to a JSON string
    string createEventMsg(string event, Json::Value &data);

    // Sends a JSON message to a spesific connected client
    void sendJsonToClient(string jsonString, u8 clientId);

    // Sends a JSON message to all connected clients
    void sendJsonToClients(string jsonString);


private:
    // WebSocket port
    unsigned short port;

    /// List of current connections
    map<u8, connection_ptr> connections;

    /// Thread for socket connection handler
    boost::shared_ptr<boost::thread> listener;

    /// Thread for running dead WebSocket remover
    boost::shared_ptr<boost::thread> cleaner;

    /// WebSocketServer endpoint ptr
    websocketpp::server *endpoint_;

    /// Framework pointer
    Framework* framework_;


    /// Socket events

    // Triggers when a client requests for a certain resource
    void on_validate(connection_ptr con);

    // Triggers when a client connects
    void on_open(connection_ptr con);

    // Triggers when a client disconnects
    void on_close(connection_ptr con);

    // Triggers when whenever a session is terminated or failed before it was successfully established
    void on_fail(connection_ptr con);

    // Triggers when a message is received
    void on_message(connection_ptr con, message_ptr msg);


    /// Utility functions
    void removeConnection(connection_ptr con);

    void cleanConnections();

    void closeConnections();

signals:

    // Emitted if we get a proper event message with data
    void gotEvent(QString event, QString data, u8 clientId);

};


} //End of namespace ColladaViewer
