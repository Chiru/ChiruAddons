// Only allow single drag&drop mesh instance.
var containerDragObject = null;
// Moveable drag&drop mesh distance from the camera.
var distanceFromCamera = 9;
var dragObjectName = "info_bubble_movable_block";

var uiCamera = null;

if (!framework.IsHeadless())
{
    engine.ImportExtension("qt.core");
    engine.ImportExtension("qt.gui");
    engine.IncludeFile("MathUtils.js");
    engine.IncludeFile("Utils.js");
    ui.GraphicsView().DragMoveEvent.connect(HandleDragMoveEvent);
    ui.GraphicsView().DropEvent.connect(HandleDropEvent);
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
            MakeDraggable(visual, widget);
        }
    }
    return visual;
}

function MakeDraggable(visual, widget)
{
	if (widget) {
		visual.DragStartEvent.connect(widget, function(e) {
			containerDragObject = ContainerDragObject(this);
	//        containerDragObject.placeable.visible = false;
	//        MoveSelected(e.pos());
			//SetInfoBoubleHighlight(true);
			// Signal object selection to the UI camera
			if (!uiCamera)
				uiCamera = scene.EntityByName("UiCamera");
			uiCamera.Exec(1, "ObjectSelected", containerDragObject != null ? containerDragObject.id.toString() : "0");
		});
	}
	else
	{
		visual.DragStartEvent.connect(function(e) {
			containerDragObject = ContainerDragObject();
			if (!uiCamera)
				uiCamera = scene.EntityByName("UiCamera");
			uiCamera.Exec(1, "ObjectSelected", containerDragObject != null ? containerDragObject.id.toString() : "0");
		});
	}

    visual.DropEvent.connect(function(e) {
        if (containerDragObject) {
            containerDragObject.placeable.visible = false;
        }
    });
    
    visual.DragMoveEvent.connect(function(e) {
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

function CreateContainerDragObject(widget)
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
    if (data.length > 0 && containerDragObject)
    {
        MoveSelected(e.pos());
    }
}

function HandleDropEvent(e)
{
    var data = e.mimeData().data("application/x-hotspot").toString();
    if (data.length > 0 && containerDragObject)
    {
        containerDragObject.placeable.visible = false;
    }

    // Signal object deselection to the UI camera
    if (!uiCamera)
        uiCamera = scene.EntityByName("UiCamera");
    uiCamera.Exec(1, "ObjectSelected", "0");
}

function CurrentMouseRay()
{
    var p = input.MousePos();
    return MouseRay(p.x(), p.y());
}

function MoveSelected(pos)
{
    if (containerDragObject)
    {
        var ray = CurrentMouseRay();
        var cameraEntity = renderer.MainCamera();
		//\Todo Invalid raycast plane calculation.
        var camFwd = cameraEntity.placeable.WorldOrientation().Mul(scene.ForwardVector()).Normalized();

        var offset = camFwd.Mul(new float3(0,0,distanceFromCamera));
        var movePlane = new Plane(cameraEntity.placeable.WorldPosition().Add(offset), camFwd);

        var r = IntersectRayPlane(movePlane, ray);
        if (r.intersects)
        {
            var moveTo = ray.GetPoint(r.distance);
            containerDragObject.placeable.SetPosition(moveTo);
        }
    }
}

function GetContainerDragObject()
{
    if (!containerDragObject)
        containerDragObject = scene.EntityByName(dragObjectName);
    return containerDragObject;
}

/*function SetInfoBoubleHighlight(active)
{
    var entity = GetContainerDragObject();
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

function ContainerDragObject(visual) 
{
    var moveEntity = GetContainerDragObject();
	
	if (visual && moveEntity && moveEntity.graphicsviewcanvas)
	{
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
	}
    moveEntity.placeable.visible = true;
    return moveEntity;
}
