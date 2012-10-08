
$(function(){

    if (!Detector.webgl)
        Detector.addGetWebGLMessage()


    var _container,  _objController,
        _sceneController = {
            loadedObjects: [], // List of downloaded collada files
            renderer: null, // WebGL render
            camera: null,
            pointLight: null,
            scene: null, // Scene hierarchy
            sceneParams: { // Parameters related to scene rendering
                'colorMode': THREE.VertexColors,
                'resolution': 1
            },
            controls: null, // Camera controls
            loader: null // Collada loader/parser
        },
        _gui = $.gui

    var _connection = {
        serverFiles: [], // List of collada files stored in remote storage
        origStorageUrl: "",
        storageUrl: "", //  Url of the remote storage folder
        wsManager: null, // WebSocket connection manager object
        ip: "",
        port: ""
    }

    function isMobileBrowser() {
        if (typeof(window.mobile) !== 'undefined')
            return window.mobile
        return false
    }

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


    function parseUrl(url) {
        var parsed = parseUri(url)

        //Parsing ip and port from address bar (uses ?ip=IP&port=PORT)
        if(parsed.queryKey){
            var keys = parsed.queryKey
            if(keys.ip)
                _connection.ip = keys.ip
            else
                _connection.ip = "127.0.0.1"

            if(keys.port)
                _connection.port = keys.port
            else
                _connection.port = "9002"

            //For testing
            if(keys.mobile)
                window.mobile = keys.mobile === '1'
            if(keys.useProxy){
                _connection.useProxy = keys.useProxy === '1'
            }else{
                _connection.useProxy = isMobileBrowser()
            }

        }
    }

    function insertToUrl(key, value) {
        key = encodeURIComponent(key); value = encodeURIComponent(value);

        var kvp = window.location.search.substr(1).split('&');
        if (kvp == ''){
            window.history.replaceState('Object', 'Title', '?' + key + '=' + value)
        }
        else{

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

    //Parsing remote storage url
    function parseStorageUrl(url){
        var parsed = parseUri(url)

        //Adding trailing slash to directory path if it is missing
        if(parsed['directory'].indexOf('/', parsed['directory'].length - 1) == -1)
            parsed['directory'] += '/'

        var proxy = "chiru.cie.fi:8000/"
        if(_connection.useProxy)
            parsed['host'] =  proxy + "?" + parsed['host']

        _connection.storageUrl =  "http://" + parsed['host'] + parsed['directory']

    }


//Function that changes the camera aspect ratio and renderer size when window is resized
    function windowResize (renderer, camera){
        var callback = function(){
            var res = _sceneController.sceneParams.resolution
            renderer.setSize(_container.innerWidth()*res, _container.innerHeight()*res)
            if(renderer.domElement.style.width || renderer.domElement.style.height){
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

        for ( var i = 0, il = children.length; i < il; i++ ) {
            setColorMode( children[ i ], colorMode  )
        }

        if ( geometry ) {
            //o3d.material.shading = THREE.FlatShading
            var mode = THREE.VertexColors
            if(colorMode == THREE.NoColors||
                colorMode == THREE.FaceColors||
                colorMode == THREE.VertexColors){
                mode = colorMode
            }
            o3d.material.vertexColors = parseInt(mode)
            o3d.material.needsUpdate = true
        }
    }

    function initWSConnection() {

        console.log("Websocket ip: " + _connection.ip + " and port: " + _connection.port)

        //Opening a websocket connection
        _connection.wsManager = new WSManager(_connection.ip, _connection.port)

        _connection.wsManager.bind("newCollada", function(colladaName){
            if(!(colladaName in _connection.serverFiles)){
                //console.log("A new collada was added in remote storage: " + colladaName)
                _connection.serverFiles.push(colladaName)

                _gui.fileList.addItem(colladaName)

                if(window.confirm("A new captured model was added to remote storage. Load the model?")){

                    requestCollada(colladaName)
                }
            }
        })

        _connection.wsManager.bind("colladaList", function(data){
            _connection.serverFiles =  data['list'].split(", ")
            _connection.serverFiles.sort()

            _connection.serverFiles.forEach(function(name) {
                _gui.fileList.addItem(name)
            })
            _connection.origStorageUrl = data['storageUrl']
            parseStorageUrl(_connection.origStorageUrl)

        })

        _connection.wsManager.bind("disconnected", function() {
            console.log("WebSocket closed.")

            //Removing all the references to the assets
            _connection.serverFiles.length = 0
            _gui.fileList.empty()

        })

    }


// Request collada through http
    function requestCollada(colladaName){
        var loadedObjects = _sceneController.loadedObjects
        for (var i=0; i < loadedObjects.length; i++){
            if(loadedObjects[i].name == colladaName){
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

                if( request.readyState == 4 ) {
                    clearInterval (trigger);
                    trigger = null
                    _objController.loading = false

                    if( request.status == 200 ) {

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
                    }

                } else if( request.readyState == 2) {
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

            if(loadedObjects.length > 0){
                //removeFromScene(loadedObjects[loadedObjects.length-1])
                clearScene()
            }

            loadedObjects.push(model)
            addToScene(model)
            _objController.setCurrent(model)

            _gui.fileList.toggleLoaded(model.name)
            model = null

            //Freeing memory by removing objects
            cleanScene()

            //console.log(_sceneController.renderer.info.memory)
            loader = null

            _gui.loadDiag.changeState('ready')

        }, _connection.storageUrl)


    }


    function addToScene(object) {
        _sceneController.scene.add(object)
    }

    function removeFromScene(object) {
        _sceneController.scene.remove(object)
    }

    function clearScene() {
        var obj, i
        var scene = _sceneController.scene

        //Removes all objects (but not the floor/camera
        for (i = scene.children.length - 1; i >= 0 ; i --) {
            obj = scene.children[i]
            if(!(obj instanceof THREE.Camera || obj instanceof THREE.Mesh || obj instanceof THREE.Light) )
                scene.remove(obj)
        }
    }

    function cleanScene() {
        var objects = _sceneController.loadedObjects
        if(objects.length > 4){
            while(objects.length > 4){
                //console.log("removing object: " + objects[0].name)

                _gui.fileList.toggleLoaded(objects[0].name)

                removeFromScene(objects[0])
                _sceneController.renderer.deallocateObject(objects[0])
                _sceneController.renderer.clear()
                objects.splice(0,1)

            }
        }
    }


//Initializes the renderer, camera, etc.
    function init() {
        parseUrl(location)
        var isMobile = isMobileBrowser()

        //Creates the container and scene
        _container = $("#content")

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

        // RENDERER

        var canvas = $("#glCanvas")

        canvas.attr({ width: _container.innerWidth(), height: _container.innerHeight()})


        //console.log("alias: " + antiAlias + " precision: " +precision + " scale: " + scale + " canvas: " + canvas.id + " w: " + canvas.width + " h: " +canvas.height)

        var renderer = _sceneController.renderer = new THREE.WebGLRenderer({
            antiAlias: true,	// to get smoother output
            preserveDrawingBuffer: false,	// true to allow screen shot
            precision: 'highp',
            canvas: canvas.get(0)
        })
        //renderer.setSize($("#content").width(), $("#content").height())
        renderer.setClearColorHex( 0xBBBBBB, 1 )

        if(isMobile)
            setRenderQuality('low')


        //Initializes object focus change controller
        _objController = new THREE.Object3D()

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


// GUI

    function guiBindings (){
        var sceneParams = _sceneController.sceneParams
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

        /*
         var f22 = _gui.rightGui.addFolder('Render options')
         f22.add(sceneParams, 'colorMode', { 'No colors': THREE.NoColors, 'VertexColors': THREE.VertexColors })
         .name('Color Mode').onFinishChange(function(){
         console.log(sceneParams.colorMode)
         setColorMode(_objController.current, sceneParams.colorMode)

         })
         */
        //Fullscreen activation key
        if(THREEx.FullScreen.available()){
            THREEx.FullScreen.bindKey({ charCode : 'f'.charCodeAt(0)})
        }

    }

//The animation loop
    function loop() {

        requestAnimationFrame(loop)
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