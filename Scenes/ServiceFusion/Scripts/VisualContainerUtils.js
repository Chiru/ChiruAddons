// !ref: InfoBubbleMovableBlock.txml

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

// Only allow single drag&drop mesh instance.
var movableInfoBubble = null;
// Moveable drag&drop mesh distance from the camera.
var distanceFromCamera = 9;

QByteArray.prototype.toString = function()
{
    ts = new QTextStream( this, QIODevice.ReadOnly );
    return ts.readAll();
}

function Container(parent)
{
    this.visual = new VisualContainer(parent);
    this.container = C3DUiModule.ContainerFactory().CreateContainer(this.visual);
    this.container.parent = parent;
    this.container.rdfStore = RdfModule.theWorld.CreateStore();
}

function CreateVisualContainer(widget, layout, parent)
{   
    var visual = new VisualContainer(parent);
    var container = C3DUiModule.ContainerFactory().CreateContainer(visual);
    container.parent = parent.owner;
    container.rdfStore = RdfModule.theWorld.CreateStore();
    
    if (parent)
    {
        if (parent.layout())
            parent.layout().addWidget(visual, 0, 0);
        else
            visual.setParent(parent);
            
        // \todo remove this when layout object has been implemented --Joosua.
        if (widget)
        {
            if (layout)
            {
                visual.setLayout(layout);
                visual.layout().addWidget(widget, null, null);
            }
            else
                widget.setParent(visual);
                
            visual.DragStart.connect(widget, function(drag) {
                movableInfoBubble = InfoBoubleMovable(this);
            });
        
            visual.DragDrop.connect(function(drag) {
                if (movableInfoBubble) {
                    scene.RemoveEntity(movableInfoBubble.id);
                    movableInfoBubble = null;
                }
            });
            
            visual.DragMove.connect(function(drag) {
                // todo add implementation --Joosua.
            });
        }
    }
    return visual;
}

function CreateMovableInfoBouble(widget)
{
    var ents = scene.LoadSceneXML(asset.GetAsset("InfoBubbleMovableBlock.txml").DiskSource(), false, false, 0);
    var entity = ents[0];
        
    var renderLabel = new QLabel();
    var pixmap =  new QPixmap(widget.size);
    widget.render(pixmap);
    renderLabel.size = widget.size;
    renderLabel.setPixmap(pixmap);
    entity.graphicsviewcanvas.width = widget.width;
    entity.graphicsviewcanvas.height = widget.height;
    
    entity.graphicsviewcanvas.GraphicsScene().addWidget(renderLabel);
    renderLabel.show();
    return entity;
}

function CreateHLine(linetype)
{
    var line = new QFrame();
    line.objectName = "line";
    line.frameShape = QFrame.HLine;
    line.frameShadow = QFrame.solid;
    return line;
}
CreateHLine.LineType = {Horizontal:0, Vertical:1};

function HandleDragMoveEvent(e)
{
    var data = e.mimeData().data("application/x-hotspot").toString();
    if (data.length > 0 && movableInfoBubble)
    {
        MoveSelected(e.pos());
    }
}

function HandleDropEvent(e)
{
    var data = e.mimeData().data("application/x-hotspot").toString();
    if (data.length > 0 && movableInfoBubble)
    {
        scene.RemoveEntity(movableInfoBubble.id);
        movableInfoBubble = null;
    }
}

function CurrentMouseRay()
{
    var x, y;
    var mousePos = input.MousePos();
    x = mousePos.x(), y = mousePos.y();
    return renderer.MainCameraComponent().GetMouseRay(x/ui.GraphicsScene().width(), y/ui.GraphicsScene().height());
}

function MoveSelected(pos)
{
    if (movableInfoBubble)
    {
        var ray = CurrentMouseRay();
        var cameraEntity = renderer.MainCamera();
        var camFwd = cameraEntity.placeable.WorldOrientation().Mul(scene.ForwardVector()).Normalized();

        var offset = camFwd.Mul(new float3(0,0,distanceFromCamera));
        var movePlane = new Plane(cameraEntity.placeable.WorldPosition().Add(offset), camFwd);

        var r = IntersectRayPlane(movePlane, ray);
        if (r.intersects)
        {
            var moveTo = ray.GetPoint(r.distance);
            movableInfoBubble.placeable.SetPosition(moveTo);
        }
    }
}

function InfoBoubleMovable(visual) 
{
    var ents = scene.LoadSceneXML(asset.GetAsset("InfoBubbleMovableBlock.txml").DiskSource(), false, false, 0);
    var moveEntity = ents[0];
    moveEntity.placeable.selectionLayer = 0;
    
    var renderLabel = new QLabel();
    var pixmap =  new QPixmap(visual.size);
    visual.render(pixmap);
    renderLabel.size = visual.size;
    renderLabel.setPixmap(pixmap);
    moveEntity.graphicsviewcanvas.width = visual.width;
    moveEntity.graphicsviewcanvas.height = visual.height;
    renderLabel.acceptDrops = false;
    
    ents[0].graphicsviewcanvas.GraphicsScene().addWidget(renderLabel);
    ents[0].graphicsviewcanvas.GraphicsView().acceptDrops = false;
    renderLabel.show();
    return moveEntity;
}

if (!framework.IsHeadless())
{
    engine.IncludeFile("MathUtils.js");
    ui.GraphicsView().DragMoveEvent.connect(HandleDragMoveEvent);
    ui.GraphicsView().DropEvent.connect(HandleDropEvent);
}