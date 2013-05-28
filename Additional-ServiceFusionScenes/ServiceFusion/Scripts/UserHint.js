engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("VisualContainerUtils.js");

var SongContainer = null;
var CalendarEventContainer= null;

var pp = null;
var Radio = null;
var Twitter = null;
var Screen = null;
var Chrome = null;
var Firefox = null;
var Youtube = null;
var entities = [];

//ui.GraphicsView().DragMoveEvent.connect(UserHints);
ui.GraphicsView().DragEnterEvent.connect(UserHints);

function UserHints(e)
{
    var data = e.mimeData().data("application/x-hotspot").toString();
    if (data.length > 0)
    {
        var ray = CurrentMouseRay();
        var r = scene.ogre.Raycast(ray, -1);

        if (SongContainer && r.entity.id == SongContainer.id)
        {
           if(Radio)
               entities.push(Radio);
           if(Twitter)
               entities.push(Twitter);
           if(Chrome)
               entities.push(Chrome);
           if(Firefox)
               entities.push(Firefox);
           if(Youtube)
               entities.push(Youtube);
           highlight();
        }

        else if (CalendarEventContainer && r.entity.id == CalendarEventContainer.id)
        {
           if(Twitter)
               entities.push(Twitter);
           if(Chrome)
               entities.push(Chrome);
           if(Firefox)
               entities.push(Firefox);
           highlight();
        }
        else if (pp && r.entity.id == pp.id)
        {
           if(Screen)
               entities.push(Screen);
           highlight();
        }

    }
}

function GetContainers()
{
    SongContainer = scene.GetEntityByName("SongContainer");
    CalendarEventContainer = scene.GetEntityByName("calendar_events");
    pp = scene.GetEntityByName("powerPoint");
}

function GetUserIcons()
{
    Radio = scene.GetEntityByName("radio_1");
    Twitter = scene.GetEntityByName("Twitter");
    Screen = scene.GetEntityByName("Cube.007");
    Chrome = scene.GetEntityByName("chrome");
    Firefox = scene.GetEntityByName("firefox");
    Youtube = scene.GetEntityByName("youtube");
}

function highlight()
{
    var i, j = 0;
    for(i=0; i<entities.length; i++)
    {
       // print("---------highlight " + entities[i].name);
        var mesh = entities[i].GetComponent("EC_Mesh");

        if(entities[i].GetComponent("EC_Material"))
            continue;
        j=0;
       // Hack to determine the length of the meshMaterial list (not exposed to script).
        while(typeof mesh.meshMaterial[j] !== "undefined") 
        {
            var input_mat = mesh.meshMaterial[j];
            var material_component = entities[i].CreateComponent("EC_Material");
            material_component.inputMat = input_mat;

            p = [];
            p[0] = "emissive=0.7 0.7 0.0 1";
            material_component.parameters = p;
            j++;
        }
    }

    ui.GraphicsView().DropEvent.connect(removeHighlight);
    //frame.Updated.connect(update);
}
/*
function update()
{
    //print("update");
    if(input.IsMouseButtonReleased(1))
    {
        print("disconnected");
        removeHighlight();
        frame.Updated.disconnect(update);
    }
}
*/

function removeHighlight()
{

    if(entities.length==0)
        return;

    var i, j = 0;
    for(i=0; i<entities.length; i++)
    {
        var mesh = entities[i].GetComponent("EC_Mesh");

            j=0;
            while(typeof mesh.meshMaterial[j] !== "undefined") 
            {
                entities[i].RemoveComponent("EC_Material");
                j++;
            }

            j=0;
            while(typeof mesh.meshMaterial[j] !== "undefined") 
            {
                var material = mesh.meshMaterial[j];
                asset.RequestAsset(material, "OgreMaterial", true);
                j++;
            }
    }
    entities = [];
    ui.GraphicsView().DropEvent.disconnect(removeHighlight);
}

GetContainers();
GetUserIcons();
