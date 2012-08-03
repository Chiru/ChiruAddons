/*
 Author: Toni Dahl

 A function for handling the WebSocket communication between Tundra server and a Web client.

 */


var Connection = function (host, port){
    var url = "ws://" + host + ":" + port;

    if ("WebSocket" in window) {
        this.ws = new WebSocket(url);
    } else if ("MozWebSocket" in window) {
        this.ws = new MozWebSocket(url);
    } else {
        document.innerHTML += "This Browser does not support WebSockets<br />";
        return;
    }

    //Storage for bound callback events
    this.callbacks = {};


    this.ws.onopen = function() {
        console.log("Connected")

        //Requesting a test collada file
        var data = {"event":"requestCollada", "data": "CapturedHorse.dae"};
        this.send(JSON.stringify(data));
    }

    this.ws.onmessage = function (evt) {
        //console.log("Got msg: " + evt.data)
        this.parseMessage(evt.data)
    }.bind(this)

    this.ws.onclose = function(evt) {
        console.log("Connection closed.")
    }

    this.ws.onerror = function(e) {
        console.log("Error:"+ e)
    }
}

Connection.prototype.parseMessage = function (message) {
    var parsed = JSON.parse(message)
    this.processEvent(parsed)
}

Connection.prototype.processEvent = function (json) {
    if(json['event']){
        console.log("Got event: "+json['event'])
        switch(json['event'])
        {
            case "loadCollada":
                if(json['data']){
                    console.log("Beginning of collada file: " + json['data'].substring(0,300)+ "...")
                    console.log("End of collada file: ..."+json['data'].substring(json['data'].length-300,json['data'].length))
                    this.triggerEvent(json['event'], json['data'])
                }
                break
            default:
                console.log("Error: Received an unknown event: " + json['event'])
        }
    }
}

//A function for binding custom event callbacks for Connection
Connection.prototype.bind = function(eventName, callback){
    this.callbacks[eventName] = this.callbacks[eventName] || [];
    this.callbacks[eventName].push(callback);
    return this;
};

//Triggers the bound event and gives it some data as argument if it has a callback function
Connection.prototype.triggerEvent = function(eventName, message){
    var eventChain = this.callbacks[eventName];
    if(typeof eventChain == 'undefined') return; //Event had no callback function
    for(var i = 0; i < eventChain.length; i++){
        eventChain[i](message)
    }
}

