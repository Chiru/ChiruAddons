engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

// Only allow single drag&drop mesh instance.
var movableInfoBubble = null;
// Moveable drag&drop mesh distance from the camera.
var distanceFromCamera = 9;
var dragObjectName = "info_bubble_movable_block";

QByteArray.prototype.toString = function()
{
    ts = new QTextStream( this, QIODevice.ReadOnly );
    return ts.readAll();
}

function BaseContainer(parent)
{
    this.visual = new VisualContainer(parent);
    this.container = new Container(this.visual);
    this.container.parent = parent;
    this.container.rdfStore = RdfModule.theWorld.CreateStore();
}

function CreateVisualContainer(widget, layout, parent)
{   
    var visual = new VisualContainer(parent);
    var container = new Container(visual);
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
            MakeDragable(visual, widget);
        }
    }
    return visual;
}

function MakeDragable(visual, widget)
{
    visual.DragStart.connect(widget, function(drag) {
        movableInfoBubble = InfoBoubleMovable(this);
        //SetInfoBoubleHighlight(true);
    });

    visual.DragDrop.connect(function(drag) {
        if (movableInfoBubble) {
            movableInfoBubble.placeable.visible = false;
        }
    });
    
    visual.DragMove.connect(function(drag) {
        // todo add implementation --Joosua.
    });
}

function AddStatement(visual, subject, predicate, object)
{
    var world = RdfModule.theWorld;
    var s = world.CreateResource(new QUrl(subject));
    var p = world.CreateResource(new QUrl(predicate));
    var o = world.CreateLiteral(object);
    var statement = world.CreateStatement(s, p, o);
    
    visual.owner.rdfStore.AddStatement(statement);
    
    world.FreeStatement(statement); 
    if (s) world.FreeNode(s);
    if (p) world.FreeNode(p);
    if (o) world.FreeNode(o);
}

function Select(store, subject, predicate, object)
{
    var world = RdfModule.theWorld;
    var s = subject ? world.CreateResource(new QUrl(subject)) : null;
    var p = predicate ? world.CreateResource(new QUrl(predicate)) : null;
    var o = object ? world.CreateLiteral(object) : null;
    var statement = world.CreateStatement(s, p, o);
    
    var statements = store.Select(statement);
    
    world.FreeStatement(statement); 
    if (s) world.FreeNode(s);
    if (p) world.FreeNode(p);
    if (o) world.FreeNode(o);
    return statements;
}

function ReleaseStatements(statements)
{
    for (var i = 0; i < statements.length; ++i)
        statements[i].deleteLater();
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

function CreateHLine()
{
    var line = new QFrame();
    line.objectName = "line";
    line.frameShape = QFrame.HLine;
    line.frameShadow = QFrame.solid;
    return line;
}

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
        movableInfoBubble.placeable.visible = false;
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

function GetMovableInfoBouble()
{
    if (!movableInfoBubble)
        movableInfoBubble = scene.EntityByName(dragObjectName);
    return movableInfoBubble;
}

/*function SetInfoBoubleHighlight(active)
{
    var entity = GetMovableInfoBouble();
    if (!entity)
        return;
        
    var material = entity.mesh.MaterialAsset(0);
    if (material)
    {
        if (active)
            material.SetEmissiveColor(0, 0, new Color(1.0, 0.0, 0.0, 1.0));
        else
            material.SetEmissiveColor(0, 0, new Color(1.0, 1.0, 1.0, 1.0));
        print(material.EmissiveColor(0, 0));
        entity.mesh.meshMaterial = [material];
    }
}*/

function InfoBoubleMovable(visual) 
{
    var moveEntity = GetMovableInfoBouble();
    var renderLabel = new QLabel();
    var pixmap =  new QPixmap(visual.size);
    visual.render(pixmap);
    renderLabel.size = visual.size;
    renderLabel.setPixmap(pixmap);
    moveEntity.graphicsviewcanvas.width = visual.width;
    moveEntity.graphicsviewcanvas.height = visual.height;
    renderLabel.acceptDrops = false;
    
    moveEntity.graphicsviewcanvas.GraphicsScene().addWidget(renderLabel);
    moveEntity.graphicsviewcanvas.GraphicsView().acceptDrops = false;
    renderLabel.show();
    moveEntity.placeable.visible = true;
    return moveEntity;
}

if (!framework.IsHeadless())
{
    engine.IncludeFile("MathUtils.js");
    ui.GraphicsView().DragMoveEvent.connect(HandleDragMoveEvent);
    ui.GraphicsView().DropEvent.connect(HandleDropEvent);
}