
//Url parser
var ip = location.hash.substr(1)
console.log("ip: " + ip)

if (!Detector.webgl)
    Detector.addGetWebGLMessage()


var camera, scene, renderer, container, loadedObjects = [], objController,
    pointLight,
    controls, gui = {},
    sceneParams = {"colorMode": THREE.VertexColors}

var connection, storageUrl = "", serverFiles = []


//Parsing remote storage url
function setStorageUrl(url){
    var parsed = parseUri(url)

    //Adding trailing slash to directory path if it is missing
    if(parsed['directory'].indexOf('/', parsed['directory'].length - 1) == -1)
        parsed['directory'] += '/'

    //Hardcoded proxy url for mobile device demos
    var proxy = "chiru.cie.fi:8000/"
    if(window.mobile)
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
        renderer.setSize(window.innerWidth, window.innerHeight)
        camera.aspect = window.innerWidth / window.innerHeight
        camera.updateProjectionMatrix()
        controls.handleResize()
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
        //o3d.material.side = THREE.DoubleSide
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
//Opening a websocket connection
    if(!ip)
        ip = "127.0.0.1"

    connection = new WSManager(ip, "9002")


    connection.bind("newCollada", function(colladaName){
        if(!(colladaName in serverFiles)){
            console.log("A new collada was added in remote storage: " + colladaName)
            serverFiles.push(colladaName)
            serverFiles.sort()

            if(window.confirm("A new captured model was added to remote storage. Load the model?")){

                requestCollada(colladaName)
            }

            var select = gui.leftGui.fileList.domElement.children[0]
            select.options[select.options.length] = new Option(colladaName, colladaName)
            console.log(serverFiles)
        }
    })

    connection.bind("colladaList", function(data){
        serverFiles =  data['list'].split(", ")
        serverFiles.sort()

        serverFiles.forEach(function(name){
            var select = gui.leftGui.fileList.domElement.children[0]
            select.options[select.options.length] = new Option(name, name)
        })
        console.log(serverFiles)

        storageUrl = setStorageUrl(data['storageUrl'])
        console.log(storageUrl)

    })

    /*
     //Binding collada loader function to websocket event
     connection.bind("loadCollada", function(data){
     console.log("loading collada")

     if(!data['collada'])
     return

     if(!data['fileName'])
     return
     var loader = new THREE.ColladaLoader()

     loader.options.convertUpAxis = true

     //Parsing the COLLADA
     loader.parse( StringtoXML(data['collada']), function colladaReady( collada ) {
     var skin = collada.skins[0],
     dae = collada.dae,
     model = collada.scene

     console.log(collada)

     //Changing colormode of the collada model
     setColorMode(model, sceneParams.colorMode)

     //console.log(collada)
     model.name = data['fileName']


     objController.loadNext = ""

     if(loadedObjects.length > 0){
     toggleVisibility(loadedObjects[loadedObjects.length -1])
     }

     loadedObjects.push(model)

     scene.add(model)

     console.log(scene)
     objController.setCurrent(model)

     gui.leftGui.objectControls.open()

     console.log(loadedObjects)

     })


     })

     */

}

/*
// Request collada through webSocket
function requestCollada(colladaName){
    for (var i=0; i < loadedObjects.length; i++){
        if(loadedObjects[i].name == colladaName){
            toggleVisibility(objController.current)
            toggleVisibility(loadedObjects[i])
            console.log(colladaName + " is loaded alredy.")
            console.log(objController.current)
            return
        }
    }
    connection.ws.send(JSON.stringify({event:"requestCollada", data:colladaName}))

}
*/

// Request collada through http
function requestCollada(colladaName){
    for (var i=0; i < loadedObjects.length; i++){
        if(loadedObjects[i].name == colladaName){
            toggleVisibility(objController.current)
            toggleVisibility(loadedObjects[i])
            console.log(colladaName + " is loaded alredy.")
            console.log(objController.current)
            return
        }
    }

    var loader = new THREE.ColladaLoader()

    loader.options.convertUpAxis = true

    loader.load(storageUrl + colladaName, function colladaReady(collada) {
        var skin = collada.skins[0],
            dae = collada.dae,
            model = collada.scene

        console.log(collada)

        //Changing colormode of the collada model
        setColorMode(model, sceneParams.colorMode)

        //console.log(collada)
        model.name = colladaName


        objController.loadNext = ""

        if(loadedObjects.length > 0){
            toggleVisibility(loadedObjects[loadedObjects.length -1])
        }

        loadedObjects.push(model)

        scene.add(model)

        console.log(scene)
        objController.setCurrent(model)

        gui.leftGui.objectControls.open()

        console.log(loadedObjects)

        showInfoMsg()

    }, function progress(data){
        showInfoMsg("Downloading model... " + " Loaded: " + Math.ceil((data.loaded / 1000000)*100)/100 + " MB")
    })


    showInfoMsg("Downloading model...")
    console.log("Requested file: " + storageUrl + colladaName)

}

function toggleVisibility(object){
    object['visible'] = !object['visible']

    THREE.SceneUtils.traverseHierarchy(object, function ( object ) {
        if (object.children === null || object.children.length == 0)
            object['visible'] = !object['visible']
    })

    if (object['visible'])
        objController.setCurrent(object)

}


//Initializes the renderer, camera, etc.
function init() {

    //Creates the container and scene
    container = document.getElementById('container')
    //document.body.appendChild( container )

    scene = new THREE.Scene()

    // Camera
    camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 1, 2000 )
    camera.position.set(17, 10, 15)
    camera.lookAt(scene.position)

    scene.add( camera )

    // Lights
    pointLight = new THREE.PointLight(0xffffff)
    pointLight.intensity = 2
    scene.add(pointLight)
    scene.add(new THREE.AmbientLight(0xffffff))

    // Scene objects
    var floorMaterial = new THREE.MeshBasicMaterial({ color :0x110000, wireframe: true, wireframeLinewidth: 1})
    var floorGeometry = new THREE.PlaneGeometry(30, 30, 20, 20)
    var floor = new THREE.Mesh(floorGeometry, floorMaterial)
    floor.position.y = -3
    floor.rotation.x = Math.PI/2
    scene.add(floor)

    // RENDERER
    renderer = new THREE.WebGLRenderer({
        antialias: true,	// to get smoother output
        preserveDrawingBuffer: false	// true to allow screenshot
    })
    renderer.setClearColorHex( 0xBBBBBB, 1 )
    renderer.setSize(window.innerWidth, window.innerHeight)
    container.appendChild(renderer.domElement)


    //Initializes object focus change controller
    objController = new THREE.Object3D()

    objController.setCurrent = function(current) {
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

    objController.loadNext = ""
    objController.loading = false

    // TRACKBALL CAMERA CONTROLS

    //passing renderer.context will pass WebGL canvas to the controls and stop them interfering with GUI
    controls = new THREE.TrackballControls(camera, renderer.domElement)
    controls.rotateSpeed = 0.8
    controls.zoomSpeed = 0.9
    controls.panSpeed = 0.4
    controls.staticMoving = false
    controls.keys = [65, 83, 68]
    controls.maxDistance = 50

    console.log(controls)

    //Windows resize listener
    windowResize(renderer, camera)

}


// GUI

function initGUI (){
    gui.leftGui = new dat.GUI({autoPlace: false})
    var leftContainer = document.getElementById('leftGui')
    leftContainer.appendChild(gui.leftGui.domElement)
    var modelScale = objController.scale
    var modelPos = objController.position
    var modelRot = objController.rotation
    modelRot.degrees = new THREE.Vector3(0.0, 0.0, 0.0)
    objController.scaleFactor = 1.0


    var f11 = gui.leftGui.addFolder('File controls')
    gui.leftGui.fileList = f11.add(objController, 'loadNext').options(serverFiles).onFinishChange(function(){
        if(objController.loadNext != "" && !objController.loading)
            requestCollada(objController.loadNext)
    })
    var select = gui.leftGui.fileList.domElement.children[0]
    select.options[select.options.length] = new Option('Choose collada', '')

    f11.open()

    var f12 = gui.leftGui.objectControls = gui.leftGui.addFolder('Object controls')
    f12.add(modelPos, 'x', -15, 15, 0.1).name('X Pos').listen().onChange(function(val){
        objController.current.position.x = val
    })
    f12.add(modelPos, 'y', -10.0, 10, 0.05).name('Y Pos').listen().onChange(function(val){
        objController.current.position.y = val
    })
    f12.add(modelPos, 'z', -15, 15, 0.1).name('Z Pos').listen().onChange(function(val){
        objController.current.position.z = val
    })
    f12.add(modelRot.degrees, 'x', -180.0, 180.0, 0.1).name('X Rot').listen().onChange(function(){
        objController.current.rotation.x = modelRot.degrees.x * Math.PI / 180
    })
    f12.add(modelRot.degrees, 'y', -180.0, 180, 0.1).name('Y Rot').listen().onChange(function(){
        objController.current.rotation.y = modelRot.degrees.y * Math.PI / 180
    })
    f12.add(modelRot.degrees, 'z', -180.0, 180, 0.1).name('Z Rot').listen().onChange(function(){
        objController.current.rotation.z = modelRot.degrees.z * Math.PI / 180
    })
    f12.add(modelScale, 'x', 0.01, 4.00, 0.01).name('Scale x').listen().onChange(function(val){
        objController.current.scale.x = val
    })
    f12.add(modelScale, 'y', 0.01, 4.00, 0.01).name('Scale y').listen().onChange(function(val){
        objController.current.scale.y = val
    })
    f12.add(modelScale, 'z', 0.01, 4.00, 0.01).name('Scale z').listen().onChange(function(val){
        objController.current.scale.z = val
    })

    gui.rightGui = new dat.GUI({autoPlace: false})
    var rightContainer = document.getElementById('rightGui')
    rightContainer.appendChild(gui.rightGui.domElement)

    var f21 = gui.rightGui.addFolder('Light settings')
    f21.add(pointLight, 'intensity', 0.1, 2).step(0.1)
    f21.open()

    var f22 = gui.rightGui.addFolder('Render options')
    f22.add(sceneParams, 'colorMode', { 'No colors': THREE.NoColors, 'VertexColors': THREE.VertexColors })
        .name('Color Mode').onFinishChange(function(){
            console.log(sceneParams.colorMode)
            setColorMode(objController.current, sceneParams.colorMode)

        })
    f22.open()
    var f23 = gui.rightGui.addFolder('Camera controls')
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

    requestAnimationFrame(loop)
    controls.update()

    render()

}


//The render function
function render() {

    pointLight.position.x = camera.position.x
    pointLight.position.y = camera.position.y
    pointLight.position.z = camera.position.z

    if(objController.current){
        pointLight.lookAt(objController.current.position)
    }

    renderer.render(scene, camera)

}


//Initialize WebSocket connection
initWSConnection()

//Initialize the WebGL renderer and scene
init()

//Initializes the GUI for modifying objects
initGUI()

//Start the animation loop
loop()

