var NumChildren = 5;
var NumGrandChildren = 2;

function GenerateTestContainerHierarchy()
{
    print("Generating container test hierarcy...");
    var vc = new VisualContainer(null);
    var mapContainer = C3DUiModule.ContainerFactory().CreateContainer(vc);
    
    for (var i = 0; i < NumChildren; ++i)
    {
        vc = new VisualContainer(null);
        var c = C3DUiModule.ContainerFactory().CreateContainer(vc);
        c.parent = mapContainer;
        for (var j = 0; j < NumGrandChildren; ++j)
        {
            vc = new VisualContainer(null);
            var child = C3DUiModule.ContainerFactory().CreateContainer(vc);
            child.Parent = c;
        }
    }
    
    return mapContainer;
}

function TestHierarchy(root)
{
    print("Testing contianer hierarcy...");
    if (root.childCount != NumChildren)
    {
        print ("Root has wrong number of children. " + root.childCount + " != " + NumChildren);
        return;
    }
    
    for (var i = 0; i < root.childCount; ++i)
    {
        var child = root.Child(i);
        if (child == null) {
            print ("Failed to get child.");
            return;
        }
        
        for (var j = 0; j < child.childCount; ++j)
        {
            grandchild = child.Child(j);
            if (grandchild == null) {
                print ("Failed to get grandchild.");
                return;
            }
        }
    }
    
    noChild = root.Child(root.ChildCount);
    noChild = root.Child(-3);
}

function ScriptTest(root)
{
    print("Testing active region and script...");
    
    var script = new TestScript();
    var script2 = new TestScript();
    
    root.eventManager.RegisterScript(new Tag("test", "test"), script);
    root.eventManager.RegisterScript(new Tag("test", ""), script2);
    
    if (!root.eventManager.HasScript(new Tag("test", "test")) || !root.eventManager.HasScript(new Tag("test", "")))
    {
        print("Non-existing script found.");
        return;
    }
    
    root.eventManager.RegisterScript(new Tag("", "noscript"), null);
    
    print("Dropping tag to active region to run test script...");
    root.DropToActive(new Tag("", "test"), root);
    root.DropToActive(new Tag("test", "teddst"), root);
    
    root.DropToActive(new Tag("", "noscript"), root);
}

var root = GenerateTestContainerHierarchy();
TestHierarchy(root);
ScriptTest(root);