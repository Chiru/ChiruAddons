/*

 A function for handling the WebSocket communication between Tundra server and a Web client.

 */

function alertBox(msg) {
    var div = document.getElementById('conAlert')
    if(msg){
        if(div){
            div.innerHTML = msg
        }else{
            var newDiv = document.createElement('div')
            newDiv.setAttribute('id','conAlert')
            newDiv.innerHTML = msg
            newDiv.style.position = 'absolute'
            newDiv.style.width = '400px'
            newDiv.style.top = '10px'
            newDiv.style.left = '50%'
            newDiv.style.marginLeft = '-200px'
            newDiv.style.fontFamily = 'Impact, Charcoal, sans-serif'
            newDiv.style.fontSize = '1.5em'
            newDiv.style.fontWeight = 'bold'
            newDiv.style.textAlign = 'center'
            newDiv.style.textShadow = '0 4px 3px rgba(0, 0, 0, 0.4), 0 8px 10px rgba(0, 0, 0, 0.1), 0 8px 9px rgba(0, 0, 0, 0.1)'
            newDiv.style.cursor ='pointer'
            newDiv.onclick = function(){newDiv.parentNode.removeChild(newDiv)}
            document.body.appendChild(newDiv)
        }
    }else{
        if(div)
            div.parentNode.removeChild(div)
    }

}

var WSManager = function (host, port, options){
    this.url = "ws://" + host + ":" + port
    this.ws = null
    this.reconnecting = false
    this.connectTimer = null

    var defaultOpt = {
        reconnectInterval: 4000,
        timeout: 5000
    }
    if(typeof(options === 'undefined'))
        options = defaultOpt

    //Reconnection interval time (milliseconds)
    if(typeof(options.reconnectInterval)==='undefined') options.reconnectInterval = defaultOpt.reconnectInterval

    this.reconnectInterval = options.reconnectInterval

    //Connection attempt timeout (milliseconds)
    if(typeof(options.timeout)==='undefined') options.timeout = defaultOpt.timeout

    this.timeout = options.timeout

    //Storage for bound callback events
    this.callbacks = {}

    var connect = function (url) {

        if ("WebSocket" in window) {
            this.ws = new WebSocket(url)
        } else if ("MozWebSocket" in window) {
            this.ws = new MozWebSocket(url)
        } else {
            alert("This Browser does not support WebSockets.  Use newest version of Google Chrome, FireFox or Opera. ")
            return
        }


        this.ws.onopen = function() {
            if(this.connectTimer != null){
                clearTimeout(this.connectTimer)
                this.connectTimer = null
            }
            console.log("WebSocket connection opened.")
            alertBox("Connected to server!")
            setTimeout(function() {alertBox()}, 2000)
            this.reconnecting = false

            //this.send(JSON.stringify({event:"requestCollada", data:"/var/lib/CapturedChair.dae"}))
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
                alertBox("WebSocket connection failed. Attempting to reconnect...")
                this.triggerEvent("disconnected")
            }
            console.log("Reconnecting in " + this.reconnectInterval/1000 + " seconds...")
            this.reconnecting = true
            this.ws = null

            setTimeout(function() {
                connect(url)
            }, this.reconnectInterval)
        }.bind(this)

        this.ws.onerror = function(e) {
            console.log(e)
            alertBox("WebSocket error: " + e.message)
        }


        //Timeout for connection attempt
        this.connectTimer = setTimeout(function() {
            this.ws.readyState = this.ws.CLOSED
            this.ws.onclose()
            this.connectTimer = null
        }.bind(this), this.timeout)

    }.bind(this)

    connect(this.url)
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

