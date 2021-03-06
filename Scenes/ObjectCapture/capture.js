engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

var data = {};
var debugMode = 0;

function ObjectCapture()
{
    print("\nStarting object capture script");
    data.module = framework.GetModuleByName("ObjectCapture");
    data.module.setRemoteStorageURL("http://chiru.cie.fi/colladaStorage");

    data.module.startCapturing();
    //Set default positions
    data.module.setLiveCloudPosition(Quat(1,0,0,0), float3(0,0,-30), float3(10,10,10));
    data.module.setGlobalModelPosition(Quat(1,0,0,0), float3(15,0,-30), float3(10,10,10));
    data.module.setFinalMeshPosition(Quat(0,0,0,0), float3(-15,0,-30), float3(10,10,10));
    
    var me = scene.GetEntityByName("ObjectCaptureApplication");
    var inputmapper = me.GetOrCreateComponent("EC_InputMapper", 2, false);
    
    inputmapper.contextPriority = 100;
    inputmapper.takeMouseEventsOverQt = false;
    inputmapper.takeKeyboardEventsOverQt = false;
    inputmapper.modifiersEnabled = false;
    inputmapper.keyrepeatTrigger = false;
    inputmapper.executionType = 1;

    inputmapper.RegisterMapping("Y", "Capture", 1); // 1 = keypress
    inputmapper.RegisterMapping("T", "rewindCloud", 1);
    inputmapper.RegisterMapping("R", "restartCapturing", 1);
    inputmapper.RegisterMapping("U", "Finalize", 1);
    inputmapper.RegisterMapping("I", "startCapturing", 1);
    inputmapper.RegisterMapping("O", "stopCapturing", 1);

    me.Action("Capture").Triggered.connect(captureCloud);
    me.Action("Finalize").Triggered.connect(finalizeCloud);
    me.Action("startCapturing").Triggered.connect(startCapturing);
    me.Action("stopCapturing").Triggered.connect(stopCapturing);
    me.Action("restartCapturing").Triggered.connect(restartCapturing);
    data.module.assetUploaded.connect(assetUploaded);

    var timer = new QTimer();
    timer.timeout.connect(this, updatePointSize);
    timer.start(250);
    data.timer = timer;

    initializePlaceholders(10);
    initializeUi();

    data.initialized = true;
}

function initializePlaceholders(numberOfPlacehorders)
{
    //print("Initializing placeholders");

    data.placeholders = new Array(numberOfPlacehorders);
    data.placeholderIndex = 0;

    var dist = 0;
    for (i = 0; i < data.placeholders.length; i++)
    {
        var placeholder = scene.CreateEntity();
        placeholder.GetOrCreateComponent("EC_Name");
        placeholder.SetName("CapturedObject"); // todo add id to name

        placeholder.GetOrCreateComponent("EC_Mesh");

        var placeable = placeholder.GetOrCreateComponent("EC_Placeable");
        placeable.SetPosition(dist * Math.pow(-1, i), 0, 30);
        placeable.SetScale(1.0, 1.0, 1.0);

        data.placeholders[i] = placeholder;
        
        if(i % 2 == 0)
            dist += 10;
    }
}

function initializeUi()
{
    var dialog = new QDialog();
    dialog.setStyleSheet("QDialog {background-color: transparent;}");
    var layout = new QHBoxLayout(dialog);
    data.dialog = dialog;

    var buttonCapture = new QPushButton("Capture");
    var buttonRewind = new QPushButton("Undo");
    var buttonExport = new QPushButton("Export");
    var buttonFinalize = new QPushButton("Finalize");
    if (debugMode == 1)
    {
        var buttonStart = new QPushButton("Start Capturing");
        var buttonStop = new QPushButton("Stop Capturing");
    }

    layout.addWidget(buttonRewind, 0, Qt.AlignLeft);
    layout.addWidget(buttonFinalize, 0, Qt.AlignCenter);
    layout.addWidget(buttonExport, 0, Qt.AlignCenter);
    layout.addWidget(buttonCapture, 0, Qt.AlignRight);
    if (debugMode == 1)
    {
        layout.addWidget(buttonStart, 0, Qt.AlignLeft);
        layout.addWidget(buttonStop, 0, Qt.AlignLeft);
    }

    var proxy = new UiProxyWidget(dialog);
    ui.AddProxyWidgetToScene(proxy);

    proxy.windowFlags = 0;
    proxy.visible = true;
    data.proxy = proxy;

    buttonCapture.clicked.connect(captureCloud);
    buttonRewind.clicked.connect(rewindCloud);
    buttonExport.clicked.connect(exportMesh);
    buttonFinalize.clicked.connect(finalizeCloud);
    if (debugMode == 1)
    {
        buttonStart.clicked.connect(startCapturing);
        buttonStop.clicked.connect(stopCapturing);
    }

    ui.GraphicsScene().sceneRectChanged.connect(onSceneResized);
    onSceneResized();
}

function onSceneResized()
{
    if (data.proxy != null)
    {
        data.proxy.width = ui.GraphicsScene().sceneRect.width() - 200;
        data.proxy.x = 100;
        data.proxy.y = ui.GraphicsScene().sceneRect.height() - 100;
    }
}

function updatePointSize()
{
    if (data.module != null)
        data.module.updatePointSize();
}

function startCapturing()
{
    print("Starting capture interface.");
    if (data.module != null)
        data.module.startCapturing();
}

function stopCapturing()
{
    print("Stopping capture interface.");
    if (data.module != null)
        data.module.stopCapturing();
}

function restartCapturing()
{
    if (!data.initialized)
        ObjectCapture();

    print ("Restarting capturing interface.");
    if (data.module != null)
        data.module.restartCapturing();
}

function captureCloud()
{
    print("Captured cloud.");
    if (data.module != null)
        data.module.captureCloud();
}

function rewindCloud()
{
    print("Cloud rewinded.");
    if (data.module != null)
        data.module.rewindCloud();
}

function finalizeCloud()
{
    print("Finalizing cloud.");
    if (data.module != null)
        data.module.finalizeCapturing();
}

function exportMesh()
{
    print ("Exporting mesh to collada.")
    if (data.module != null)
        data.module.exportCollada("CapturedObject.dae"); // todo ask filename from user
    //print ("Export finished!");
}

function OnScriptDestroyed()
{
    if (data.proxy != null)
    {
        data.proxy.close();
        data.proxy.deleteLater();
        data.proxy = null;
    }

    if (data.dialog != null)
    {
        data.dialog.close();
        data.dialog.deleteLater();
        data.dialog = null;
    }

    if (data.timer != null)
    {
        data.timer.stop();
        data.timer.deleteLater();
        data.timer = null;
    }

    if (data.placeholders != null)
    {
        // Prevent multiple placeholders creation if script is reloaded.
        /// \todo Move creation of placeholders to server side so that capturing client can be restarted without placeholder spamming.
        for (i = 0; i < data.placeholders.length; i++)
        {
            scene.RemoveEntity(data.placeholders[i].Id());
        }
        data.placeholders = null;
    }

    stopCapturing();
    data.module = null;
}

function getNextPlaceholder()
{
    var tmpvar;
    if (data.placeholders != null)
    {
        if (data.placeholderIndex < data.placeholders.length)
        {
            print ("Placeholder index is: " +data.placeholderIndex);
            tmpvar = data.placeholders[data.placeholderIndex];
        }
        else
        {
            data.placeholderIndex = 0;
            tmpvar = data.placeholders[data.placeholderIndex];
        }
        data.placeholderIndex++;
        return tmpvar;
    }
    else
    {
        print("placeholders object is null!");
    }
}

function assetUploaded(assetRef)
{
    var placeholder = getNextPlaceholder();
    if (placeholder != null)
    {
        var mesh = placeholder.GetComponent("EC_Mesh");
        mesh.SetMeshRef(assetRef);

        var assetRef = mesh.meshMaterial;
        assetRef[0] = "local://CapturedObject.material";
        mesh.meshMaterial = assetRef;
    }

    else
    {
        print("ObjectCaptureScript: Error: placeholder is null!");
    }
}

if (!server.IsRunning())
{
    // Add Capture menu to the client Ui
    var menu = ui.MainWindow().menuBar();
    var captureMenu = menu.addMenu("&Capture");
    captureMenu.addAction("Start ObjectCapture").triggered.connect(InitializeObjectCapture);
    captureMenu.addAction("Restart Capturing").triggered.connect(restartCapturing);

    data.initialized = false;
}

function InitializeObjectCapture()
{
    ObjectCapture();
}
