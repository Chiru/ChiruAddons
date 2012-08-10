#include "WebSocketManager.h"
#include <boost/version.hpp>

namespace ColladaViewer
{

WebSocketManager::WebSocketManager(unsigned short port)
{
    this->port = port;

}

WebSocketManager::~WebSocketManager()
{
    stopServer();
}


/// Socket events

void WebSocketManager::on_validate(connection_ptr con)
{
    cout << "Request for resource: " << con->get_resource() << endl;
    cout << "origin: " << con->get_origin() << endl;

}

void WebSocketManager::on_open(connection_ptr con)
{
    cout << "Web Client " << con << " connected." << endl;

    //Adding the new connection to the list of connections
    connections.insert(pair<string, connection_ptr>(getConId(con), con));
    cout << "Web clients connected: " << connections.size() << endl;

    /*
    for ( std::map<connection_ptr,std::string>::const_iterator iter = connections.begin();
       iter != connections.end(); ++iter )
       std::cout << iter->first << '\t' << iter->second << '\n';

    std::cout << std::endl;
    */

}

void WebSocketManager::on_close(connection_ptr con)
{
    cout << "Web Client " << con << " disconnected." << endl;

    map<string, connection_ptr>::iterator it = connections.find(getConId(con));

    if (it == connections.end()) {
        /* This client has already disconnected, we can ignore this.
        This happens when there is a deliberate "soft" disconnection
        preceeding the "hard" socket read fail or disconnect ack message. */
        return;
    }

    connections.erase(it);

}

void WebSocketManager::on_message(connection_ptr con, message_ptr msg)
{
    cout << "Got message: " << msg->get_payload() << endl;
    ptree pt;

    //Parses the message as JSON
    if(parseJson(msg->get_payload(), pt) != -1) {

        //Checking if JSON string has event information and data
        QString event = QString::fromStdString(pt.get<string>("event", "empty"));

        if (event == "empty")
            return;

        QString data = QString::fromStdString(pt.get<string>("data", "empty"));

        if (data == "empty")
            return;

        emit gotEvent(event, data, QString::fromStdString(getConId(con)));
    }

}


/// Utility functions

string WebSocketManager::getConId(connection_ptr con)
{
    stringstream endpt;
    //endp << con->get_endpoint();
    endpt << con;
    return endpt.str();
}

map<string, server::connection_ptr>::iterator WebSocketManager::findClient(string clientId)
{
    map<string, server::connection_ptr>::iterator it = connections.find(clientId);
    return it;
}

int WebSocketManager::parseJson(string s, ptree &pt)
{
    try
    {
        stringstream ss;
        ss << s;
        boost::property_tree::json_parser::read_json(ss, pt);
        return 0;
    }
    catch(const boost::property_tree::json_parser::json_parser_error& e)
    {
        cerr << "WSManager JSON parse error: " << e.what() << endl;
        return -1;
    }

}


string WebSocketManager::createEventMsg(string event, string data)
{
    ptree json;
    json.put<string>("event", event);
    json.put<string>("data", data);

    stringstream buffer;
    write_json(buffer, json);

    return buffer.str();
}


void WebSocketManager::sendJsonToClient(string jsonString, string clientId)
{
    map<string, server::connection_ptr>::iterator it = findClient(clientId);
    it->second->send(jsonString);
}

void WebSocketManager::sendJsonToClients(string jsonString)
{
    for ( std::map<string, server::connection_ptr>::const_iterator iter = connections.begin();
          iter != connections.end(); ++iter )
        iter->second->send(jsonString);

}

void WebSocketManager::startServer()
{
    cout << "Boost version used: " << BOOST_LIB_VERSION << endl;
    cout << "Starting WebSocket server on port " << port << endl;

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

        //Getting pointer to the right function
        void(websocketpp::role::server<websocketpp::server>::*f)(uint16_t,size_t) =
                &websocketpp::role::server<websocketpp::server>::listen;

        cout << "Starting WSServer thread... \n";
        boost::shared_ptr<boost::thread> ptr(new boost::thread(f, endpoint_, port, 1));
        t = ptr;
        //t->detach();


    } catch (exception& e) {
        cerr << "WebSocketManager Exception: " << e.what() << endl;
    }

}

void WebSocketManager::stopServer()
{
    cout << "Stopping WebSocket server on port " << port << endl;

    try {
        endpoint_->stop();
        if(t->joinable()){
            t->join();
            t.reset();
        }

    } catch (exception& e) {
        cerr << "WebSocketManager Exception: " << e.what() << endl;
    }

}

} //End of namespace ColladaViewer




