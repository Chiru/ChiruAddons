/*

 A function for handling the WebSocket communication between Tundra server and a Web client.

 */

var WSManager = function (host, port, options){
    var defaultOpt = {
        reconnectInterval: 4000,
        timeout: 5000
    }

    this.url = "ws://" + host + ":" + port
    this.ws = null
    this.reconnecting = false
    this.connectTimer = null
    this.reconnectTimer = null
    this.reconnAttempt = 0

    if(typeof(options) === 'object')
        options = $.extend(defaultOpt, options)
    else
        options = defaultOpt

    //Reconnection interval time (milliseconds)
    this.reconnectInterval = options.reconnectInterval

    //Connection attempt timeout (milliseconds)
    this.timeout = options.timeout

    //Storage for bound callback events
    this.callbacks = {}

    this.connect = function () {
        if ("MozWebSocket" in window) {
            this.ws = new MozWebSocket(this.url)
        } else if ("WebSocket" in window) {
            this.ws = new WebSocket(this.url)
        } else {
            alert("This Browser does not support WebSockets.  Use newest version of Google Chrome, FireFox or Opera. ")
            return
        }

        this.ws.onopen = function() {
            if(this.connectTimer != null){
                clearTimeout(this.connectTimer)
                this.connectTimer = null
                this.reconnAttempt = 0
            }
            this.triggerEvent("connected", host+":"+port)
            this.reconnecting = false
            this.reconnectTimer = null
        }.bind(this)

        this.ws.onmessage = function (evt) {
            //console.log("Got msg: " + evt.data)
            this.parseMessage(evt.data)
        }.bind(this)

        this.ws.onclose = function(evt) {

            if(this.connectTimer != null){
                clearTimeout(this.connectTimer)
                this.connectTimer = null
            }

            if(!this.reconnecting){
                this.triggerEvent("disconnected", host+":"+port)
            }

            console.log("Reconnecting in " + this.reconnectInterval/1000 + " seconds...")
            this.reconnecting = true
            this.ws = null

            this.reconnectTimer = setTimeout(function() {
                this.reconnAttempt = this.reconnAttempt + 1
                this.triggerEvent("reconnecting", {host: host+":"+port, attempt: this.reconnAttempt})
                this.connect(this.url)
            }.bind(this), this.reconnectInterval)

        }.bind(this)

        this.ws.onerror = function(e) {
            console.log(e.data)
            this.triggerEvent("error", e.data)
        }

        //Timeout for connection attempt
        this.connectTimer = setTimeout(function() {
            this.ws.readyState = this.ws.CLOSED
            this.ws.onclose()
            this.connectTimer = null
        }.bind(this), this.timeout)

        return false;

    }.bind(this)

}

WSManager.prototype.open = function() {
    this.connect(this.url)
}

WSManager.prototype.stopReconnect = function() {
    clearTimeout(this.reconnectTimer)
    this.reconnecting = false
}

WSManager.prototype.parseMessage = function (message) {
    var parsed = JSON.parse(message)
    this.processEvent(parsed)
}

WSManager.prototype.processEvent = function (json) {
    if(json['event']){
        console.log("Got event: "+json['event'])
        if(json['data']){

            // Triggering the event
            this.triggerEvent(json['event'], json['data'])
        }
    }
}

//A function for binding custom event callbacks for Connection
WSManager.prototype.bind = function(eventName, callback){
    this.callbacks[eventName] = this.callbacks[eventName] || []
    this.callbacks[eventName].push(callback)
    return this;
};

//Triggers the bound event and gives it some data as argument if it has a callback function
WSManager.prototype.triggerEvent = function(eventName, message){
    var eventChain = this.callbacks[eventName];
    if(typeof eventChain == 'undefined'){
        console.log("Error: Received an unknown event: " + eventName)
        return;
    }
    for(var i = 0; i < eventChain.length; i++){
        eventChain[i](message)
    }
}

