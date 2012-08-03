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

//#include "ObjectCaptureModule.h"

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
    void startServer();
    void stopServer();

    void sendUserConnected();

    /// Utility functions
    string getConId(connection_ptr con);
    map<string, server::connection_ptr>::iterator findClient(string clientId);
    int parseJSON(string s, ptree&pt);
    string createEventMsg(string event, string data);
    void sendJsonToClient(string jsonString, string clientId);


private:
    unsigned short port;
    /// List of current connections
    map<string, connection_ptr> connections;

    /// Thread for socket connection handler
    boost::shared_ptr<boost::thread> t;

    /// WebSocketServer endpoint ptr
    websocketpp::server *endpoint_;

    /// Socket events
    void on_validate(connection_ptr con);
    void on_open(connection_ptr con);
    void on_close(connection_ptr con);
    void on_message(connection_ptr con, message_ptr msg);

signals:
    void gotEvent(QString event, QString data, QString clientId);

};

}
