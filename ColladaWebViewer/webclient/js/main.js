
if (!Detector.webgl)
    Detector.addGetWebGLMessage()


var _container,  _objController,
    _sceneController = {
        loadedObjects: [], // List of downloaded collada files
        renderer: null, // WebGL render
        camera: null,
        pointLight: null,
        scene: null, // Scene hierarchy
        sceneParams: {'colorMode': THREE.VertexColors, resolution: 1}, // Parameters related to scene rendering
        controls: null, // Camera controls
        loader: null // Collada loader/parser
    },
    _gui = {}

var _connection = {
    serverFiles: [], // List of collada files stored in remote storage
    storageUrl: "", //  Url of the remote storage folder
    wsManager: null // WebSocket connection manager object
}

function isMobileBrowser() {
    if(typeof(window.mobile) !== 'undefined')
        return window.mobile
    return false
}

//Parsing remote storage url
function setStorageUrl(url){
    var parsed = parseUri(url)

    //Adding trailing slash to directory path if it is missing
    if(parsed['directory'].indexOf('/', parsed['directory'].length - 1) == -1)
        parsed['directory'] += '/'

    //Hardcoded proxy url for mobile device demos
    var useProxy = false
    if(typeof(_connection.useProxy) === 'undefined'){
        useProxy = isMobileBrowser()
    }else{
        useProxy = _connection.useProxy
    }

    var proxy = "chiru.cie.fi:8000/"
    if(useProxy)
        parsed['host'] =  proxy + "?" + parsed['host']

    return "http://" + parsed['host'] + parsed['directory']

}

function showInfoMsg(msg) {
    var msgElement = document.getElementById("loading")

    if(msgElement){
        if(msg){
            msgElement.innerHTML = msg

            if(msgElement.style.display = "none")
                msgElement.style.display = "block"
        }else{
            msgElement.innerHTML = ""
            msgElement.style.display = "none"
        }
    }
}


//Function that changes the camera aspect ratio and renderer size when window is resized
function windowResize (renderer, camera){
    var callback = function(){
        var factor = _sceneController.sceneParams.resolution
        renderer.setSize(window.innerWidth*factor, window.innerHeight*factor)
        camera.aspect = (window.innerWidth*factor) / (window.innerHeight*factor)
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
        console.log("gotMode: " + colorMode + "set mode: " +mode)
        o3d.material.vertexColors = parseInt(mode)
        o3d.material.needsUpdate = true
        //console.log(o3d)
    }
}

/*
//Converts the XML data from string to XML object
function StringtoXML(string){
    if (window.ActiveXObject){
        var doc = new ActiveXObject('Microsoft.XMLDOM')
        doc.async = 'false'
        doc.loadXML(string)
    } else {
        var parser = new DOMParser()
        var doc = parser.parseFromString(string,'text/xml')
    }
    return doc
}
*/

function initWSConnection() {
    var ip = "", port = ""
    var parsedAddress = parseUri(location)

    //Parsing ip and port from address bar (uses ?ip=IP&port=PORT)
    if(parsedAddress.queryKey){
       var keys = parsedAddress.queryKey
        if(keys.ip){
            ip = keys.ip
        }else{
            ip = "127.0.0.1"
        }
        if(keys.port){
            port = keys.port
        }else{
            port = "9002"
        }

        //For testing
        if(keys.mobile)
            window.mobile = keys.mobile === '1'
        if(keys.useProxy)
            _connection.useProxy = keys.useProxy === '1'

    }

    console.log("Websocket ip: " + ip + " and port: " + port)

    //Opening a websocket connection
    _connection.wsManager = new WSManager(ip, port)

    _connection.wsManager.bind("newCollada", function(colladaName){
        if(!(colladaName in _connection.serverFiles)){
            console.log("A new collada was added in remote storage: " + colladaName)
            _connection.serverFiles.push(colladaName)
            _connection.serverFiles.sort()

            if(window.confirm("A new captured model was added to remote storage. Load the model?")){

                requestCollada(colladaName)
            }

            var select = _gui.leftGui.fileList.domElement.children[0]
            select.options[select.options.length] = new Option(colladaName, colladaName)
            console.log(_connection.serverFiles)
        }
    })

    _connection.wsManager.bind("colladaList", function(data){
        _connection.serverFiles =  data['list'].split(", ")
        _connection.serverFiles.sort()

        var select = _gui.leftGui.fileList.domElement.children[0]
        _connection.serverFiles.forEach(function(name){
            select.options.add(new Option(name, name))
        })

        _connection.storageUrl = setStorageUrl(data['storageUrl'])

    })

    _connection.wsManager.bind("disconnected", function() {
        console.log("WebSocket closed.")

        //Removing all the references to the assets
        _connection.serverFiles.length = 0
        var select = _gui.leftGui.fileList.domElement.children[0]
        while(select.options.length > 1) {
            select.options[1] = null
        }
    })
}


// Request collada through http
function requestCollada(colladaName){
    var loadedObjects = _sceneController.loadedObjects
    for (var i=0; i < loadedObjects.length; i++){
        if(loadedObjects[i].name == colladaName){
            console.log(colladaName + " is loaded alredy.")
            removeFromScene(_objController.current)
            addToScene(loadedObjects[i])
            _objController.setCurrent(loadedObjects[i])

            return
        }
    }

    // Collada loader/parser
    showInfoMsg("Requesting model...")

    var loader = new THREE.ColladaLoader()

    loader.options.convertUpAxis = true

    _objController.loading = true
    loader.load(_connection.storageUrl + colladaName, function colladaReady(collada) {
        var model = collada.scene

        //Changing colormode of the collada model
        setColorMode(model, _sceneController.sceneParams.colorMode)

        model.name = colladaName

        //Forcing double-sideness
        THREE.SceneUtils.traverseHierarchy(model, function(child) {
            child.material.side = THREE.DoubleSide
        })

        _objController.loadNext = ""
        _objController.loading = false

        if(loadedObjects.length > 0){
            //removeFromScene(loadedObjects[loadedObjects.length-1])
            clearScene()
            console.log("removed earlier object from scene")
        }

        loadedObjects.push(model)
        addToScene(model)
        _objController.setCurrent(model)


        var select = _gui.leftGui.fileList.domElement.children[0].options
        for (var i = 0; i < select.length; i++) {
            if(select[i].label == model.name){
                select[i].style.backgroundColor = "lightGreen"
                break
            }
        }

        model = null

        //Freeing memory by removing objects
        cleanScene()

        _gui.leftGui.objectControls.open()

        //console.log(_sceneController.renderer.info.memory)
        showInfoMsg()

        loader = null

    }, function progress(data){
        showInfoMsg("Downloading model... " + " Loaded: " + Math.ceil((data.loaded / 1000000)*100)/100 + " MB")
    })



    console.log("Requested file: " + _connection.storageUrl + colladaName)



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
            console.log("removing object: " +objects[0].name)

            var select = _gui.leftGui.fileList.domElement.children[0].options
            for (var i = 0; i < select.length; i++) {
                if(select[i].label == objects[0].name){
                    select[i].style.backgroundColor = null
                    console.log(select[i].label)
                    break
                }
            }

            removeFromScene(objects[0])
            _sceneController.renderer.deallocateObject(objects[0])
            _sceneController.renderer.clear()
            objects.splice(0,1)

        }
    }
}


//Initializes the renderer, camera, etc.
function init() {
    var isMobile = isMobileBrowser()

    //Creates the container and scene
    _container = document.getElementById('container')

    var scene = _sceneController.scene = new THREE.Scene()
    var scale = isMobile ? 0.5 : 1

    // Camera
    var camera = _sceneController.camera = new THREE.PerspectiveCamera( 45, (window.innerWidth * scale) / (window.innerHeight * scale), 1,70 )
    camera.position.set(17, 10, 15)
    camera.lookAt(scene.position)

    scene.add( camera )

    // Lights
    var pointLight = _sceneController.pointLight = new THREE.PointLight(0xffffff)
    pointLight.intensity = 2
    scene.add(pointLight)
    scene.add(new THREE.AmbientLight(0xffffff))

    // Scene objects
    /*
    var floorMaterial = new THREE.MeshBasicMaterial({ color :0x110000, wireframe: true, wireframeLinewidth: 1})
    var floorGeometry = new THREE.PlaneGeometry(30, 30, 20, 20)
    var floor = new THREE.Mesh(floorGeometry, floorMaterial)
    floor.position.y = -3
    floor.rotation.x = Math.PI/2
    console.log(floor)
    floor.matrixAutoUpdate = false
    floor.updateMatrix()
    scene.add(floor)
    */
    // RENDERER

    var antiAlias = isMobile ? false : true
    var precision = isMobile ? 'lowp' : 'highp'

    _sceneController.sceneParams.resolution = scale
    var canvas = document.getElementById("glCanvas")

    if(isMobile)
       canvas.id = "mobileCanvas"

    canvas.width = window.innerWidth * scale
    canvas.height = window.innerHeight * scale

    //console.log("alias: " + antiAlias + " precision: " +precision + " scale: " + scale + " canvas: " + canvas.id + " w: " + canvas.width + " h: " +canvas.height)

    var renderer = _sceneController.renderer = new THREE.WebGLRenderer({
        antiAlias: antiAlias,	// to get smoother output
        preserveDrawingBuffer: false,	// true to allow screen shot
        precision: precision,
        canvas: canvas
    })

    renderer.setClearColorHex( 0xBBBBBB, 1 )

    _container.appendChild(renderer.domElement)


    //Initializes object focus change controller
    _objController = new THREE.Object3D()

    _objController.setCurrent = function(current) {
        this.current = current
        if (this.current) {
            this.position.x = current.position.x
            this.position.y = current.position.y
            this.position.z = current.position.z
            this.scale.x = current.scale.x
            this.scale.y = current.scale.y
            this.scale.z = current.scale.z
            this.rotation.x = current.rotation.x
            this.rotation.y = current.rotation.y
            this.rotation.z = current.rotation.z
            this.rotation.degrees.x = current.rotation.x / (Math.PI / 180)
            this.rotation.degrees.y = current.rotation.y / (Math.PI / 180)
            this.rotation.degrees.z = current.rotation.z / (Math.PI / 180)
        }
    }

    _objController.loadNext = ""
    _objController.loading = false


    // TRACKBALL CAMERA CONTROLS

    //passing renderer.context will pass WebGL canvas to the controls and stop them interfering with GUI
    var controls = _sceneController.controls = new THREE.TrackballControls(camera, renderer.domElement)
    controls.rotateSpeed = 0.7
    controls.zoomSpeed = 0.9
    controls.panSpeed = 0.4
    controls.staticMoving = false
    controls.keys = [65, 83, 68]
    controls.maxDistance = 50

    //Windows resize listener
    windowResize(_sceneController.renderer, _sceneController.camera)

}


// GUI

function initGUI (){
    _gui.leftGui = new dat.GUI({autoPlace: false})
    var leftContainer = document.getElementById('leftGui');
    leftContainer.appendChild(_gui.leftGui.domElement)
    var modelScale = _objController.scale
    var modelPos = _objController.position
    var modelRot = _objController.rotation
    var sceneParams = _sceneController.sceneParams
    var pointLight = _sceneController.pointLight
    var camera = _sceneController.camera

    modelRot.degrees = new THREE.Vector3(0.0, 0.0, 0.0)
    _objController.scaleFactor = 1.0


    var f11 = _gui.leftGui.addFolder('File controls')
    _gui.leftGui.fileList = f11.add(_objController, 'loadNext').options(_connection.serverFiles).onFinishChange(function(){
        if(_objController.loadNext != "" && !_objController.loading)
            requestCollada(_objController.loadNext)
    })
    var select = _gui.leftGui.fileList.domElement.children[0]
    select.options[select.options.length] = new Option('Choose collada', '')

    f11.open()

    var f12 = _gui.leftGui.objectControls = _gui.leftGui.addFolder('Object controls')
    f12.add(modelPos, 'x', -15, 15, 0.1).name('X Pos').onChange(function(val){
        _objController.current.position.x = val
    })
    f12.add(modelPos, 'y', -10.0, 10, 0.1).name('Y Pos').onChange(function(val){
        _objController.current.position.y = val
    })
    f12.add(modelPos, 'z', -15, 15, 0.1).name('Z Pos').onChange(function(val){
        _objController.current.position.z = val
    })
    f12.add(modelRot.degrees, 'x', -180.0, 180.0, 0.1).name('X Rot').onChange(function(){
        _objController.current.rotation.x = modelRot.degrees.x * Math.PI / 180
    })
    f12.add(modelRot.degrees, 'y', -180.0, 180, 0.1).name('Y Rot').onChange(function(){
        _objController.current.rotation.y = modelRot.degrees.y * Math.PI / 180
    })
    f12.add(modelRot.degrees, 'z', -180.0, 180, 0.1).name('Z Rot').onChange(function(){
        _objController.current.rotation.z = modelRot.degrees.z * Math.PI / 180
    })
    f12.add(modelScale, 'x', 0.1, 4, 0.1).name('Scale x').onChange(function(val){
        _objController.current.scale.x = val
    })
    f12.add(modelScale, 'y', 0.1, 4, 0.1).name('Scale y').onChange(function(val){
        _objController.current.scale.y = val
    })
    f12.add(modelScale, 'z', 0.1, 4, 0.1).name('Scale z').onChange(function(val){
        _objController.current.scale.z = val
    })

    _gui.rightGui = new dat.GUI({autoPlace: false})
    var rightContainer = document.getElementById('rightGui')
    rightContainer.appendChild(_gui.rightGui.domElement)

    var f21 = _gui.rightGui.addFolder('Light settings')
    f21.add(pointLight, 'intensity', 0.1, 2).step(0.1)
    f21.open()

    var f22 = _gui.rightGui.addFolder('Render options')
    f22.add(sceneParams, 'colorMode', { 'No colors': THREE.NoColors, 'VertexColors': THREE.VertexColors })
        .name('Color Mode').onFinishChange(function(){
            console.log(sceneParams.colorMode)
            setColorMode(_objController.current, sceneParams.colorMode)

        })
    f22.open()
    var f23 = _gui.rightGui.addFolder('Camera controls')
    f23.add(camera.position, 'x', -50,50,0.1).name('X Pos').listen()
    f23.add(camera.position, 'y', -50,50,0.1).name('Y Pos').listen()
    f23.add(camera.position, 'z', -50,50,0.1).name('Z Pos').listen()
    f23.open()

    //Fullscreen activation key
    if(THREEx.FullScreen.available()){
        THREEx.FullScreen.bindKey({ charCode : 'f'.charCodeAt(0)})
    }

}

//The animation loop
function loop() {

    window.requestAnimationFrame(loop)
    _sceneController.controls.update()

    render()

}


//The render function
function render() {
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


//Initialize WebSocket connection
initWSConnection()

//Initialize the WebGL renderer and scene
init()

//Initializes the GUI for modifying objects
initGUI()

//Start the animation loop
loop()

