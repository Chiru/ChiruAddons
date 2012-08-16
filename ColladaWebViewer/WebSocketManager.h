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

#include "StableHeaders.h"


namespace ColladaViewer
{

using websocketpp::server;
using boost::property_tree::ptree;

using namespace std;


class WebSocketManager : public QObject, public server::handler
{

    Q_OBJECT

public:
    WebSocketManager(unsigned short port);
    ~WebSocketManager();

    // Starts the websocket server and event listeners
    void startServer();

    // Gracefully stops the server
    void stopServer();



    /// Utility functions

    // Creates a client id from an unique connection pointer
    string getConId(connection_ptr con);

    // Finds a connected client by client id
    map<string, server::connection_ptr>::iterator findClient(string clientId);

    // Parses a JSON string and creates a ptree from the JSON data
    int parseJson(string s, ptree&pt);

    // Creates an event message and converts it to a JSON string
    string createEventMsg(string event, ptree &data);

    // Sends a JSON message to a spesific connected client
    void sendJsonToClient(string jsonString, string clientId);

    // Sends a JSON message to all connected clients
    void sendJsonToClients(string jsonString);


private:
    unsigned short port;
    /// List of current connections
    map<string, connection_ptr> connections;

    /// Thread for socket connection handler
    boost::shared_ptr<boost::thread> t;

    /// WebSocketServer endpoint ptr
    websocketpp::server *endpoint_;


    /// Socket events

    // Triggers when a client requests for a certain resource
    void on_validate(connection_ptr con);

    // Triggers when a client connects
    void on_open(connection_ptr con);

    // Triggers when a client disconnects
    void on_close(connection_ptr con);

    // Triggers when a message is received
    void on_message(connection_ptr con, message_ptr msg);

signals:

    // Emitted if we get a proper event message with data
    void gotEvent(QString event, QString data, QString clientId);

};


} //End of namespace ColladaViewer
