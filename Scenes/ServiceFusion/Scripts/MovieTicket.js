engine.IncludeFile("MovieStyleSheets.js");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");

me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);
me.Action("SetTicketInfo").Triggered.connect(SetTicketInfo);
me.Action("SetTicketInfo2").Triggered.connect(SetTicketInfo2);
//frame.Updated.connect(Update);

if (!server.IsRunning() && !framework.IsHeadless())
{
    engine.ImportExtension("qt.core");
    engine.ImportExtension("qt.gui");
}

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
}



var placeable = me.GetOrCreateComponent("EC_Placeable");
placeable.visible = false;
placeable.SetParent(scene.EntityByName("UiCamera"), 0);
placeable.SetScale(1, 0.4, 1);
placeable.SetPosition(-12, -6, -20.0);
//placeable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / 1.2));
var mesh = me.GetOrCreateComponent("EC_Mesh");
var canvas = me.GetOrCreateComponent("EC_GraphicsViewCanvas");
canvas.submesh = 1;


mesh.meshRef = new AssetReference("news_tablet.mesh");
var materials = mesh.meshMaterial;
materials = ["ticket.material", "screen.material"];
mesh.meshMaterial = materials;

if (me.graphicsviewcanvas)
{
    var content = me.GetComponent("EC_GraphicsViewCanvas", "Content");
    var title = me.GetComponent("EC_GraphicsViewCanvas", "Title");
    if (content && title)
    {
        content.outputTexture = asset.GenerateUniqueAssetName("Texture", "ScreenContent");
        title.outputTexture =asset.GenerateUniqueAssetName("Texture", "ScreenTitle");
    }
    else
    {
        me.graphicsviewcanvas.outputTexture = asset.GenerateUniqueAssetName("Texture", "Screen");
    }
}

var frame_ticket = new QFrame();
frame_ticket.objectName = "Ticket";

var TicketStyle = "QFrame#Ticket { padding: 0px; border: 0px; border-radius: 0px; border-image: url(../src/ChiruAddons/Scenes/ServiceFusion/Assets/Finnkino/Elokuvalippu3.png); }";



frame_ticket.setStyleSheet(TicketStyle);

var grid = new QGridLayout();
grid.setContentsMargins(20, 20, 20, 20);
var ticket_text = "";
var ticket = new QLabel("Elokuvalippu");
ticket.setStyleSheet(LargeText);
ticket.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding);

var label_name = new QLabel();
var label_time = new QLabel();
var label_date = new QLabel();
var label_place = new QLabel();
var label_row = new QLabel();
var label_seat = new QLabel();
label_name.setStyleSheet(NormalText);
label_time.setStyleSheet(NormalText);
label_date.setStyleSheet(NormalText);
label_place.setStyleSheet(NormalText);
label_row.setStyleSheet(NormalText);
label_seat.setStyleSheet(NormalText);

grid.addWidget(ticket, 0, 0, 1, 2);

frame_ticket.setLayout(grid);


function SetTicketInfo(name, time, date)
{
    label_name.setText(name);
    label_time.setText(time);
    label_date.setText(date);
    
    grid.addWidget(label_name, 1, 0);
    grid.addWidget(label_time, 2, 0);
    grid.addWidget(label_date, 2, 1);

    //This causes crash
    //if (scene.EntityByName("MoviePaymentDialog")) 
    //    scene.RemoveEntity(scene.EntityByName("MoviePaymentDialog").id); 
}

function SetTicketInfo2(place, row, seat)
{
    label_place.setText(place);
    label_row.setText("Rivi " + row);
    label_seat.setText("Paikka " + seat);
    //ticket_text = ticket_text + "\n" + place + " Rivi " + row + " Paikka " + seat;
    grid.addWidget(label_place, 3, 0);
    grid.addWidget(label_row, 3, 1);
    grid.addWidget(label_seat, 3, 2);
    frame_ticket.setLayout(grid);
    //grid.addWidget(label_ticketinfo, 1, 0);
    me.graphicsviewcanvas.GraphicsScene().addWidget(frame_ticket);  
    me.graphicsviewcanvas.width = frame_ticket.width;
    me.graphicsviewcanvas.height = frame_ticket.height;
    frame_ticket.show();
    placeable.visible = true;
}










