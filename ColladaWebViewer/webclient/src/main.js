
$(function(){

    if (!Detector.webgl)
        Detector.addGetWebGLMessage()

    var _container,  _objController = new THREE.Object3D(),
        _sceneController = {
            loadedObjects: [], // List of downloaded collada files
            renderer: null, // WebGL renderer
            camera: null,
            pointLight: null,
            scene: null, // ThreeJS Scene
            sceneParams: { // Parameters related to scene rendering
                'colorMode': THREE.VertexColors, // Default color mode for loaded collada files
                'resolution': 1 // Default WebGL canvas resolution
            },
            controls: null // Camera controls
        },
        _gui = $.gui

    var _connection = {
        serverFiles: [], // List of collada files stored in remote storage
        origStorageUrl: "", // Original & unchanged storage url
        storageUrl: "", //  Url of the remote storage folder (this will change depending if we use proxy or not)
        wsManager: null, // WebSocket connection manager object
        ip: "127.0.0.1", // WebSocket server ip
        port: "9002" // WebSocket server port
    }

    // Checks if the browser is running on mobile device (the real checking is done in detectmobilebrowser.js)
    function isMobileBrowser() {
        if (typeof(window.mobile) !== 'undefined')
            return window.mobile
        return false
    }

    // Sets the WebGL canvas resolution (5-10 fps render speed increase with lower resolution)
    function setRenderQuality(quality) {
        if (typeof(quality) === 'undefined')
            quality = 'high'

        var renderer = _sceneController.renderer

        if (quality == 'high') {
            renderer.setSize( _container.innerWidth() , _container.innerHeight() )
            renderer.domElement.style.width = null
            renderer.domElement.style.height = null
            _sceneController.sceneParams.resolution = 1
        }else{
            renderer.setSize( _container.innerWidth() / 2, _container.innerHeight() / 2 )
            renderer.domElement.style.width = _container.innerWidth() + 'px'
            renderer.domElement.style.height = _container.innerHeight() + 'px'
            _sceneController.sceneParams.resolution = 0.5
        }
    }


    // Gets the url from browser's address-bar and parses it
    function parseUrl(url) {
        var parsed = parseUri(url)

        //Parsing ip and port from address bar (uses ?ip=IP&port=PORT)
        if (parsed.queryKey){
            var keys = parsed.queryKey
            if (keys.ip)
                _connection.ip = keys.ip

            if (keys.port)
                _connection.port = keys.port

            //For testing
            if (keys.mobile)
                window.mobile = keys.mobile === '1'
            if (keys.useProxy){
                _connection.useProxy = keys.useProxy === '1'
            }else{
                _connection.useProxy = isMobileBrowser()
            }
        }
    }

    // Does changes to url in browser address bar
    function insertToUrl(key, value) {
        key = encodeURIComponent(key); value = encodeURIComponent(value);

        var kvp = window.location.search.substr(1).split('&');
        if (kvp == ''){
            window.history.replaceState('Object', 'Title', '?' + key + '=' + value)

        }else{
            var i = kvp.length; var x; while (i--){
                x = kvp[i].split('=');

                if (x[0] == key) {
                    x[1] = value;
                    kvp[i] = x.join('=');
                    break;
                }
            }

            if (i < 0) { kvp[kvp.length] = [key, value].join('='); }

            window.history.replaceState('Object', 'Title', '?'+kvp.join('&'))
        }
    }

    // Parses remote storage url
    function parseStorageUrl(url){
        var parsed = parseUri(url)

        //Adding trailing slash to directory path if it is missing
        if (parsed['directory'].indexOf('/', parsed['directory'].length - 1) == -1)
            parsed['directory'] += '/'

        // Setting up hardcoded proxy for demo stuff
        var proxy = "chiru.cie.fi:8000/"
        if (_connection.useProxy)
            parsed['host'] =  proxy + "?" + parsed['host']

        _connection.storageUrl =  "http://" + parsed['host'] + parsed['directory']

    }


    //Function that changes the camera aspect ratio and renderer size when window is resized
    function windowResize (renderer, camera){
        var callback = function(){
            var res = _sceneController.sceneParams.resolution
            renderer.setSize(_container.innerWidth()*res, _container.innerHeight()*res)
            if (renderer.domElement.style.width || renderer.domElement.style.height){
                renderer.domElement.style.width = _container.innerWidth() + 'px'
                renderer.domElement.style.height = _container.innerHeight() + 'px'
            }

            camera.aspect = ((_container.innerWidth()*res) / (_container.innerHeight()*res))
            camera.updateProjectionMatrix()
            _sceneController.controls.handleResize()
        }
        window.addEventListener('resize', callback, false)

        return {
            stop: function(){
                window.removeEventListener('resize', callback)
            }
        }
    }

    function setColorMode (o3d, colorMode) {

        var children = o3d.children, geometry = o3d.geometry

        for (var i = 0, il = children.length; i < il; i++) {
            setColorMode( children[ i ], colorMode  )
        }

        if (geometry) {
            //o3d.material.shading = THREE.FlatShading
            var mode = THREE.VertexColors
            if (colorMode == THREE.NoColors||
                colorMode == THREE.FaceColors||
                colorMode == THREE.VertexColors){
                mode = colorMode
            }
            o3d.material.vertexColors = parseInt(mode)
            o3d.material.needsUpdate = true
        }
    }

    // Initializing the WebSocket and WebSocket events
    function initWSConnection() {
        console.log("Websocket ip: " + _connection.ip + " and port: " + _connection.port)

        //Opening a websocket connection
        _connection.wsManager = new WSManager(_connection.ip, _connection.port)

        // Binding events
        _connection.wsManager.bind("newCollada", function(colladaName){
            if (!(colladaName in _connection.serverFiles)){
                //console.log("A new collada was added in remote storage: " + colladaName)
                _connection.serverFiles.push(colladaName)

                _gui.fileList.addItem(colladaName)

                _gui.confirmDiag.dialog( "open" )
                _gui.confirmDiag.fileName.text(colladaName)
            }
        })

        _connection.wsManager.bind("colladaList", function(data){
            _connection.serverFiles.length = 0
            _gui.fileList.empty()

            _connection.serverFiles =  data['list'].split(", ")
            _connection.serverFiles.sort()

            _connection.serverFiles.forEach(function(name) {
                _gui.fileList.addItem(name)
            })
            _connection.origStorageUrl = data['storageUrl']
            parseStorageUrl(_connection.origStorageUrl)

        })

        _connection.wsManager.bind("connected", function(url) {
            console.log("WebSocket connection opened.")
            _gui.warnings.displayMsg("WebSocket connection opened to: " + url, {showTime: 5000})
        })

        _connection.wsManager.bind("disconnected", function(e) {
            console.log("WebSocket closed.")
            _gui.warnings.displayMsg("WebSocket connection to " + e.url + " " + e.reason + ".", {type:'warning'})

        })

        _connection.wsManager.bind("reconnecting", function(e) {
            _gui.warnings.displayMsg("Attempting to reconnect to " + e.host + " (Attempt: " + e.attempt + ")", {type:'warning'})

        })

        _connection.wsManager.bind("error", function(e) {
            _gui.warnings.displayMsg("WebSocket error: " + e,  {type:'error'})
        })

        // Opening the WebSocket connection
        _connection.wsManager.connect()

    }


    // Requests a collada using XMLHTTPRequest
    function requestCollada(colladaName){
        var loadedObjects = _sceneController.loadedObjects
        for (var i=0; i < loadedObjects.length; i++){
            if (loadedObjects[i].name == colladaName){
                //console.log(colladaName + " is loaded alredy.")
                removeFromScene(_objController.current)
                addToScene(loadedObjects[i])
                _objController.setCurrent(loadedObjects[i])

                return
            }
        }

        // Collada loader/parser
        var url = _connection.storageUrl + colladaName

        _gui.loadDiag.changeState('request', _connection.storageUrl)

        var trigger;

        if ( document.implementation && document.implementation.createDocument ) {

            var request = new XMLHttpRequest();
            // Linking request with gui, so it can be aborted
            _gui.loadDiag.xhr = request

            request.onreadystatechange = function() {

                if ( request.readyState == 4 ) {
                    clearInterval (trigger);
                    trigger = null
                    _objController.loading = false

                    if ( request.status == 200 ) {

                        if ( request.responseXML ) {
                            _gui.loadDiag.changeState('downloaded')

                            //The final download progress update
                            _gui.loadDiag.updateProgress(Math.ceil((request.responseText.length / 1000024)*100)/100)

                            //Gives the program some time to breath after download so it has time to update the viewport
                            setTimeout(function(){processCollada(request.responseXML, colladaName)}, 500)

                        } else {
                            _gui.loadDiag.changeState('error', "Empty XML received!")
                            request = null
                        }
                    } else if ( request.status == 404 ) {
                        _gui.loadDiag.changeState('error', "File not found from: " + url)
                        request = null
                    }

                } else if ( request.readyState == 2) {
                    _gui.loadDiag.changeState('loading')
                    _objController.loading = true
                    trigger = setInterval (function ()
                    {
                        if (request.readyState == 3)
                        {
                            _gui.loadDiag.updateProgress(Math.ceil((request.responseText.length / 1000024)*100)/100)
                        }
                    }, 200)
                }
            }

            request.onabort = function () {
                clearInterval (trigger);
                trigger = null
                request = null
            }

            request.onerror = function (e) {
                clearInterval (trigger);
                trigger = null
                request = null
                _gui.loadDiag.changeState('error', "Failed to download: " +url)
            }

            request.open( "GET", url, true );
            try{
                request.send( null );
            }catch (e){
                _gui.loadDiag.changeState('error', e.message+", when requesting: " +url)
            }

        } else {
            _gui.loadDiag.changeState('error', "Your browser can't handle XML.")
        }
    }

    // Parses the collada file and adds it to the scene
    function processCollada (xml, name) {
        var loadedObjects = _sceneController.loadedObjects
        var loader = new THREE.ColladaLoader()

        loader.options.convertUpAxis = true
        loader.parse(xml, function colladaReady(collada) {
            var model = collada.scene
            //Changing colormode of the collada model
            setColorMode(model, _sceneController.sceneParams.colorMode)

            model.name = name

            //Forcing double-sideness
            THREE.SceneUtils.traverseHierarchy(model, function(child) {
                child.material.side = THREE.DoubleSide
            })

            // Removing earlier object from scene
            if (loadedObjects.length > 0){
                clearScene()
            }

            // Adding the new object to memory and in scene
            loadedObjects.push(model)
            addToScene(model)
            _objController.setCurrent(model)
            _gui.fileList.toggleLoaded(model.name)
            model = null

            //Freeing memory by removing and de-allocating unneeded objects
            cleanMemory()

            //console.log(_sceneController.renderer.info.memory)
            loader = null

            _gui.loadDiag.changeState('ready')

        }, _connection.storageUrl)


    }

    // Adds a ThreeJS object to the scene
    function addToScene(object) {
        _sceneController.scene.add(object)
    }

    // Removes a ThreeJS object from the scene
    function removeFromScene(object) {
        _sceneController.scene.remove(object)
    }


    // Removes all the objects from the scene (but does not de-allocate)
    function clearScene() {
        var obj, i
        var scene = _sceneController.scene

        //Removes all objects (but not the floor/camera)
        for (i = scene.children.length - 1; i >= 0 ; i --) {
            obj = scene.children[i]
            if (!(obj instanceof THREE.Camera || obj instanceof THREE.Mesh || obj instanceof THREE.Light) )
                scene.remove(obj)
        }
    }

    // Removes unneeded objects from scene and de-allocates them
    function cleanMemory(freeMemory, cleanAll) {
        if (typeof(freeMemory) === 'undefined')
            freeMemory = true
        if (typeof(cleanAll) === 'undefined')
            cleanAll = false

        var objects = _sceneController.loadedObjects

        if (freeMemory) {

            var len = 4
            if (cleanAll)
                len = 0

            if (objects.length > len){
                while(objects.length > len){
                    //console.log("removing object: " + objects[0].name)

                    removeFromScene(objects[0])

                    _sceneController.renderer.deallocateObject(objects[0])

                    _gui.fileList.toggleLoaded(objects[0].name)
                    objects.splice(0,1)

                }
            }
            _sceneController.renderer.clear()

        }else{
            objects.forEach(function(object) {
                _sceneController.renderer.deallocateObject(object)
            })
            _sceneController.renderer.clear()
        }
    }


    //Initializes the renderer, camera, etc.
    function init(reInit) {
        if (typeof(reInit) === 'undefined')
            reInit = false

        if (reInit){
            _gui.warnings.displayMsg("Reinitialing lost WebGL context...", {type:'warning', showTime: 2000})
            // Deallocating all objects from GPU and freeing loaded objects
            cleanMemory(true, true)
            _sceneController.renderer = null
        }

        parseUrl(location)
        var isMobile = isMobileBrowser()

        var canvas = $("#glCanvas")

        // Initializing scene, camera and light (init only once)
        if (!reInit){
            // Inits the canvas
            _container = $("#content")
            canvas.attr({ width: _container.innerWidth(), height: _container.innerHeight()})

            var scene = _sceneController.scene = new THREE.Scene()

            // Camera
            var camera = _sceneController.camera = new THREE.PerspectiveCamera( 45, (_container.innerWidth() / _container.innerHeight()), 1,70 )
            camera.position.set(17, 10, 15)
            camera.lookAt(scene.position)

            scene.add( camera )

            // Lights
            var pointLight = _sceneController.pointLight = new THREE.PointLight(0xffffff)
            pointLight.intensity = 2
            scene.add(pointLight)
            scene.add(new THREE.AmbientLight(0xffffff))


            // WebGL context-change listeners
            canvas.get(0).addEventListener("webglcontextlost", function(event) {
                event.preventDefault()
            }, false)

            canvas.get(0).addEventListener(
                // Reinitializing the WebGL stuff if context is restored
                "webglcontextrestored", function(){init(true)}, false)

        }


        // Init RENDERER
        var renderer = _sceneController.renderer = new THREE.WebGLRenderer({
            antiAlias: true,	// to get smoother output
            preserveDrawingBuffer: false,	// true to allow screen shot
            precision: 'highp',
            canvas: canvas.get(0)
        })
        //renderer.setSize($("#content").width(), $("#content").height())
        renderer.setClearColorHex( 0xBBBBBB, 1 )

        if (isMobile)
            setRenderQuality('low')

        // Initializes controls and controllers (init only once)
        if (!reInit){
            //Initializes object focus change controller

            _objController.setCurrent = function(current) {
                this.current = current
                if (this.current) {
                    this.scale.x = current.scale.x
                    this.scale.y = current.scale.y
                    this.scale.z = current.scale.z
                }
            }

            _objController.loading = false


            // TRACKBALL CAMERA CONTROLS

            //passing renderer.context will pass WebGL canvas to the controls and stop them interfering with GUI
            var controls = _sceneController.controls = new THREE.TrackballControls(camera, renderer.domElement)
            controls.staticMoving = false
            controls.maxDistance = 50
            controls.minDistance = 5
            controls.rotateSpeed = 0.7
            controls.zoomSpeed = 0.9
            controls.panSpeed = 0.4
            controls.keys = [65, 83, 68]

            //Windows resize listener
            windowResize(_sceneController.renderer, _sceneController.camera)
        }
    }


    // GUI BINDINGS (Binds webgl stuff to gui controls)
    function guiBindings (){
        var pointLight = _sceneController.pointLight

        _gui.fileList.setCallback(function(name) {
            if(!_objController.loading)
                requestCollada(name)
        })

        _gui.objectScale.scaleX.bind('changed', function(event, value) {
            _objController.current.scale.x = value
        })
        _gui.objectScale.scaleY.bind('changed', function(event, value) {
            _objController.current.scale.y = value
        })
        _gui.objectScale.scaleZ.bind('changed', function(event, value) {
            _objController.current.scale.z = value
        })

        _gui.sceneParams.lightIntensity.setValue(pointLight.intensity)
        _gui.sceneParams.lightIntensity.bind('changed', function(event, value){
            pointLight.intensity = value
        })

        if(_connection.useProxy){
            _gui.sceneParams.useProxy.attr('checked','checked');
            _gui.sceneParams.useProxy.button("refresh")
        }

        _gui.sceneParams.useProxy.bind('click', function(){
            if(!_connection.useProxy)
                insertToUrl('useProxy', 1)
            else
                insertToUrl('useProxy', 0)

            parseUrl(location)
            if(_connection.origStorageUrl.length !=0)
                parseStorageUrl(_connection.origStorageUrl)
        })

        if(isMobileBrowser()){
            _gui.sceneParams.renderQuality.attr('checked','checked');
            _gui.sceneParams.renderQuality.button('refresh')
        }
        _gui.sceneParams.renderQuality.bind('click', function(){
            if(!window.mobile)
                insertToUrl('mobile', 1)
            else
                insertToUrl('mobile', 0)

            parseUrl(location)

            if(isMobileBrowser())
                setRenderQuality('low')
            else
                setRenderQuality('high')

        })

        _gui.confirmDiag.bind('ok', function(){
            requestCollada(_gui.confirmDiag.fileName.text())
        })

        //Fullscreen activation key
        if(THREEx.FullScreen.available()){
            THREEx.FullScreen.bindKey({ charCode : 'f'.charCodeAt(0)})
        }

    }

    //The animation loop
    function loop() {
        _sceneController.controls.update()

        var pointLight = _sceneController.pointLight
        var camera = _sceneController.camera
        pointLight.position.x = camera.position.x
        pointLight.position.y = camera.position.y
        pointLight.position.z = camera.position.z

        if(_objController.current){
            pointLight.lookAt(_objController.current.position)
        }

        _sceneController.renderer.render(_sceneController.scene, camera)

        requestAnimationFrame(loop)
    }

//Initialize the WebGL renderer and scene
    init()

//Binds the gui to the WebGL control values
    guiBindings()

//Initialize WebSocket connection
    initWSConnection()

//Start the animation loop
    loop()

})