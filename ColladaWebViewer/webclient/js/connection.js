/*

 A function for handling the WebSocket communication between Tundra server and a Web client.

 */

function alertBox(msg) {
    var div = document.getElementById('conAlert')
    if(msg){
        if(div){
            div.innerHTML = msg
        }else{
            var newDiv = document.createElement('div');
            newDiv.setAttribute('id','conAlert');
            newDiv.innerHTML = msg;
            newDiv.style.position = 'absolute'
            newDiv.style.width = '400px'
            newDiv.style.top = '10px'
            newDiv.style.left = '50%'
            newDiv.style.marginLeft = '-200px'
            newDiv.style.fontFamily = 'Impact, Charcoal, sans-serif'
            newDiv.style.fontSize = '25px'
            newDiv.style.fontWeight = 'bold'
            newDiv.style.textAlign = 'center'
            newDiv.style.textShadow = '0 4px 3px rgba(0, 0, 0, 0.4), 0 8px 10px rgba(0, 0, 0, 0.1), 0 8px 9px rgba(0, 0, 0, 0.1)'
            newDiv.style.cursor ='pointer'
            newDiv.onclick = function(){newDiv.parentNode.removeChild(newDiv)}
            document.body.appendChild(newDiv)
        }
    }else{
        if(div)
            document.body.removeChild(div)
    }

}

var Connection = function (host, port, reconnectInterval){
    this.url = "ws://" + host + ":" + port
    this.ws = null
    this.reconnecting = false

    //Reconnection interval time (milliseconds)
    if(typeof(reconnectInterval)==='undefined') reconnectInterval = 4000

    this.reconnectInterval = reconnectInterval
    console.log(this.reconnectInterval)

    //Storage for bound callback events
    this.callbacks = {};

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
            if(!this.reconnecting)
                alertBox("WebSocket connection closed. Attempting to reconnect...")
            console.log("Reconnecting in " + this.reconnectInterval/1000 + " seconds...")
            this.reconnecting = true
            this.ws = null
            setTimeout(function() {
                connect(url)
            }, this.reconnectInterval)
        }.bind(this)

        this.ws.onerror = function(e) {
            console.log("Error:"+ e)
        }
    }.bind(this)

    connect(this.url)
}


Connection.prototype.parseMessage = function (message) {
    var parsed = JSON.parse(message)
    this.processEvent(parsed)
}

Connection.prototype.processEvent = function (json) {
    if(json['event']){
        console.log("Got event: "+json['event'])
        if(json['data']){

            // Triggering the event
            this.triggerEvent(json['event'], json['data'])
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
    if(typeof eventChain == 'undefined'){
        console.log("Error: Received an unknown event: " + eventName);
    }
    for(var i = 0; i < eventChain.length; i++){
        eventChain[i](message)
    }
}

