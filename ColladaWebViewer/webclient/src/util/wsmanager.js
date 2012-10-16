/*

 A function for handling the WebSocket communication between Tundra server and a Web client.

 */

var WSManager = function (host, port, options){
    var defaultOpt = {
        reconnectInterval: 8000,
        timeout: 8000
    }
    this.ws = null
    this.url = "ws://" + host + ":" + port
    this.host = host
    this.port = port

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

    // Hack for firefox
    window.addEventListener("beforeunload", function() {
        this.ws.onclose = function () {} // disable onclose handler
        this.ws.close()
        this.ws = null
    }.bind(this), false)

}


WSManager.prototype.connect = function () {
    this.ws = null

    if ("WebSocket" in window) {
        this.ws = new WebSocket(this.url)
    }else if ("MozWebSocket" in window) {
        this.ws = new MozWebSocket(this.url)
    } else {
        alert("This Browser does not support WebSockets.  Use newest version of Google Chrome, FireFox or Opera. ")
        return
    }

    //Timeout for connection attempt NOTE: Disabled due abnormal behaviour with Firefox on Android
    /*
     this.connectTimer = setTimeout(function() {
     if(this.ws != null)
     if(this.ws.readyState != this.ws.OPEN)
     this.ws.close()
     }.bind(this), this.timeout)
     */

    this.ws.onopen = function() {
        //clearTimeout(this.connectTimer)
        //this.connectTimer = null

        clearTimeout(this.reconnectTimer)
        this.reconnectTimer = null

        this.reconnAttempt = 0
        this.reconnecting = false

        this.triggerEvent("connected", this.host+":" + this.port)

    }.bind(this)

    this.ws.onmessage = function (evt) {
        //console.log("Got msg: " + evt.data)
        this.parseMessage(evt.data)
    }.bind(this)

    this.ws.onclose = function(evt) {
        //clearTimeout(this.connectTimer)
        //this.connectTimer = null

        clearTimeout(this.reconnectTimer)
        this.reconnectTimer = null

        if(!this.reconnecting){
            var reason = "failed"
            if(evt.wasClean)
                reason = "closed"

            this.triggerEvent("disconnected", {url: this.host + ":" + this.port, reason: reason, code: evt.code})
        }

        // Reconnect if the  connection was not closed cleanly (network error/abnormal server close etc.)
        if(!evt.wasClean){
            this.reconnecting = true

            console.log("Reconnecting in " + this.reconnectInterval/1000 + " seconds...")

            this.reconnectTimer = setTimeout(function() {
                if(this.ws != null){
                    if(this.ws.readyState != this.ws.OPEN || this.ws.readyState != this.ws.CONNECTING){
                        this.reconnAttempt = this.reconnAttempt + 1
                        this.triggerEvent("reconnecting", {host: this.url, attempt: this.reconnAttempt})

                        this.connect(this.url)
                    }
                }
            }.bind(this), this.reconnectInterval)
        }

    }.bind(this)

    this.ws.onerror = function(e) {
        this.triggerEvent("error", e)
    }
}

WSManager.prototype.stopReconnect = function() {
    if(this.reconnectTimer != null)
        clearTimeout(this.reconnectTimer)

    if(this.connectTimer != null)
        clearTimeout(this.connectTimer)

    this.reconnecting = false
}

WSManager.prototype.parseMessage = function (message) {
    var parsed = JSON.parse(message)
    this.processEvent(parsed)
}

WSManager.prototype.processEvent = function (json) {
    if(json['event']){
        //console.log("Got event: "+json['event'])
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

