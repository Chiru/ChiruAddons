//Free Sans
me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);
frame.Updated.connect(Update);

if (!server.IsRunning() && !framework.IsHeadless())
{
    engine.ImportExtension("qt.core");
    engine.ImportExtension("qt.gui");
}

var row = -1;
var SeatInformationArray = [];
var rotation = 1.0;
var rotation_2 = 1.2;

var placeable = me.GetOrCreateComponent("EC_Placeable");
placeable.SetParent(scene.EntityByName("UiCamera"), 0);
placeable.SetScale(2, 0.75, 1);
placeable.SetPosition(0, -4, -12.0);
placeable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / 1.2));
var mesh = me.GetOrCreateComponent("EC_Mesh");
var canvas = me.GetOrCreateComponent("EC_GraphicsViewCanvas");
canvas.submesh = 1;


mesh.meshRef = new AssetReference("news_tablet.mesh");
var materials = mesh.meshMaterial;
materials = ["news_tablet.material", "screen.material"];
mesh.meshMaterial = materials;

var loginEnt = scene.EntityByName("MovieLoginDialog");
var seatEnt = scene.EntityByName("MovieSeatDialog");
var paymentEnt = scene.EntityByName("MoviePaymentDialog");
var loginPlaceable = loginEnt.GetOrCreateComponent("EC_Placeable");
var seatPlaceable = seatEnt.GetOrCreateComponent("EC_Placeable");
var paymentPlaceable = paymentEnt.GetOrCreateComponent("EC_Placeable");

var ProceedBackward = false;
var ProceedForward = false;

me.Action("SeatSelection").Triggered.connect(SeatSelection);

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
var frame_seatselection = new QFrame();
var seats_added = false;

var frame_noseats = new QFrame();
frame_noseats.objectName = "NoSeats";
frame_noseats.setFrameStyle(QFrame.StyledPanel);
frame_noseats.setStyleSheet("QFrame#NoSeats { padding: 0px; border: 2px solid black; border-radius: 0px; border-image: url(../src/ChiruAddons/Scenes/ServiceFusion/Assets/Finnkino/plaza_3.png); }");

me.graphicsviewcanvas.GraphicsScene().addWidget(frame_noseats);
me.graphicsviewcanvas.width = frame_noseats.width;
me.graphicsviewcanvas.height = frame_noseats.height;
frame_noseats.show();

SeatSelection();
SetEmptyPlaza();




function CreateSeat(row, number)
{

    var seat = new QCheckBox();
    seat.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed);
    seat.setContentsMargins(0, 0, 0, 0);
    seat.setStyleSheet("QCheckBox { padding: 0px; } QCheckBox::indicator { width: 30px; height: 38px; } QCheckBox::indicator:unchecked { border-image: url(../src/ChiruAddons/Scenes/ServiceFusion/Assets/Finnkino/green_seat.png); } QCheckBox::indicator:checked { border-image: url(../src/ChiruAddons/Scenes/ServiceFusion/Assets/Finnkino/yellow_seat.png); }");
    var seat_info = [seat, row, number];
    SeatInformationArray.push(seat_info);
    return seat;
}

function CreateHandicapSeat()
{

    var seat = new QCheckBox();
    seat.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed);
    seat.setContentsMargins(0, 0, 0, 0);
    seat.setStyleSheet("QCheckBox { padding: 0px; } QCheckBox::indicator { width: 30px; height: 38px; } QCheckBox::indicator:unchecked { border-image: url(../src/ChiruAddons/Scenes/ServiceFusion/Assets/Finnkino/wheelchair.png); } QCheckBox::indicator:checked { border-image: url(../src/ChiruAddons/Scenes/ServiceFusion/Assets/Finnkino/wheelchair.png); }");
    return seat;
}

function CreateRowLayout()
{
    var row = new QHBoxLayout();
    row.setSpacing(0);
    row.setContentsMargins(0,0,0,0);
    return row;
}

function CreateStep(size, number)
{
    if (size == 0)
    {
        var label_step = new QLabel("" + number);
        label_step.setStyleSheet("QLabel { border-image: url(../src/ChiruAddons/Scenes/ServiceFusion/Assets/Finnkino/step_small.png); color: white; font-size: 16px; qproperty-alignment: AlignCenter; }");
        label_step.setFixedSize(67, 37);
        label_step.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred);
        return label_step;
    }
    else if (size == 1)
    {
        var label_step = new QLabel("" + number);
        label_step.setStyleSheet("QLabel { border-image: url(../src/ChiruAddons/Scenes/ServiceFusion/Assets/Finnkino/step_medium.png); color: white; font-size: 16px; qproperty-alignment: AlignCenter; }");
        label_step.setFixedSize(84, 37);
        label_step.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred);
        return label_step;
    }
    else if (size == 2)
    {
        var label_step = new QLabel("" + number);
        label_step.setStyleSheet("QLabel { border-image: url(../src/ChiruAddons/Scenes/ServiceFusion/Assets/Finnkino/step_large.png); color: white; font-size: 16px; qproperty-alignment: AlignCenter; }");
        label_step.setFixedSize(101, 37);
        label_step.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred);
        return label_step;
    }
    else if (size == 3)
    {
        var label_step = new QLabel("" + number);
        label_step.setStyleSheet("QLabel { color: white; font-size: 16px; qproperty-alignment: AlignCenter; }");
        label_step.setFixedSize(101, 37);
        label_step.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred);
        return label_step;
    }
    else
        print("Give step size (0-3) when creating a step.");
}

function SeatSelection()
{
    //me.graphicsviewcanvas.GraphicsScene().clear();
    if (seats_added)
    {
        frame_noseats.hide();
        me.graphicsviewcanvas.width = frame_seatselection.width;
        me.graphicsviewcanvas.height = frame_seatselection.height;
        frame_seatselection.show();
        return;
    }
    frame_seatselection.objectName = "Seats";
    frame_seatselection.setFrameStyle(QFrame.StyledPanel);
    frame_seatselection.setStyleSheet("QFrame#Seats { padding: 0px; border: 2px solid black; border-radius: 0px; border-image: url(../src/ChiruAddons/Scenes/ServiceFusion/Assets/Finnkino/plaza_2.png); }");
    
    var buttonOK = new QPushButton("Seuraava");
    buttonOK.clicked.connect(OKClicked);
    var buttonCancel = new QPushButton("Peruuta");
    buttonCancel.clicked.connect(CancelClicked);
    var rowsLayout = new QVBoxLayout();
    rowsLayout.setContentsMargins(165,0,165,0);
    var row_array = [];
    var real_seat_number = 0;
    
    for (var i = 1; i <= 16; i++)
    {    
        row_array.push(CreateRowLayout());
    }

    

    
    //Add first row (0)
    for (var seat_number = 1; seat_number <=34; seat_number++)
    {
        row_array[0].addWidget(CreateSeat(15, seat_number), 1, Qt.AlignCenter); 
	row_array[0].setContentsMargins(0, 130, 0, 0);
    }

    //Add rows 1-7
    for (var row_number = 1; row_number <= 7; row_number++)
    {

        for (var seat_number = 1; seat_number <=32; seat_number++)
        {
            real_seat_number++;

            if (seat_number == 3 || seat_number == 30) //Add stair steps
            {
                
                row_array[row_number].addWidget(CreateStep(0, (15-row_number)), 1, 0);
		real_seat_number--;
            }
                
            else
            {
                row_array[row_number].addWidget(CreateSeat((15-row_number), real_seat_number), 1, Qt.AlignCenter); 
            }
        }
	real_seat_number = 0;
    }
    real_seat_number = 0;

    //Add row 7
    for (var seat_number = 1; seat_number <=31; seat_number++)
    {
	real_seat_number++;
        if (seat_number == 3 || seat_number == 29) //Add stair steps
        {
            
            row_array[8].addWidget(CreateStep(1, 7), 1, 0);
	    real_seat_number--;
        }
            
        else
        {
            row_array[8].addWidget(CreateSeat(7, real_seat_number), 1, Qt.AlignCenter); 
        }
    }

    real_seat_number = 0;

    //Add rows 8-13
    for (var row_number = 9; row_number <= 13; row_number++)
    {
        row_array[row_number].setContentsMargins(73,0,73,0);
        for (var seat_number = 1; seat_number <= 26; seat_number++)
        {
	    real_seat_number++;
            if (seat_number == 1 || seat_number == 26) //Add stair steps
            {
                
                row_array[row_number].addWidget(CreateStep(2, (6 + 9 - row_number)), 1, 0);
		real_seat_number--;
            }
                
            else
            {
                row_array[row_number].addWidget(CreateSeat((6 + 9 - row_number), real_seat_number), 1, Qt.AlignCenter); 
            }
        }
	real_seat_number = 0;
    }

    real_seat_number = 0;

    //Add last row
    row_array[14].setContentsMargins(73,0,73,0);
    for (var seat_number = 1; seat_number <= 24; seat_number++)
    {
	real_seat_number++;
        if (seat_number == 1 || seat_number == 24) //Add stair steps
	{
            row_array[14].addWidget(CreateStep(3, 1), 1 , 0);
	    real_seat_number--;
	}
        else if ((seat_number > 1 && seat_number < 5) || (seat_number > 20 && seat_number < 24))
	{
            row_array[14].addWidget(CreateHandicapSeat(), 1, Qt.AlignCenter);
	    real_seat_number--;
	}
        else
	{
            row_array[14].addWidget(CreateSeat(1, real_seat_number), 1, Qt.AlignCenter);
	}
    }

    //Add buttons
    row_array[15].addWidget(buttonCancel, 1, Qt.AlignLeft);
    row_array[15].addWidget(buttonOK, 1, Qt.AlignRight);
    row_array[15].setContentsMargins(0, 150, 0, 180);
    
    
    for (var i = 0; i <= 15; i++)
        rowsLayout.addLayout(row_array[i]);
    
    frame_seatselection.setLayout(rowsLayout);
    me.graphicsviewcanvas.GraphicsScene().addWidget(frame_seatselection);
    me.graphicsviewcanvas.width = frame_seatselection.width;
    me.graphicsviewcanvas.height = frame_seatselection.height;
    frame_seatselection.hide();

    row = -1;
    seats_added = true;
}

function SetEmptyPlaza()
{    
    frame_seatselection.hide();
    me.graphicsviewcanvas.width = frame_noseats.width;
    me.graphicsviewcanvas.height = frame_noseats.height;
    frame_noseats.show();
}

function OKClicked()
{
    var seat_found = false;
    for (var seat = 0; seat < SeatInformationArray.length; seat++)
    {
    	if (SeatInformationArray[seat][0].checked)
        {
    	    console.LogInfo("Selected seat found. Row: " + SeatInformationArray[seat][1] + " Number: " + SeatInformationArray[seat][2]);
            seat_found = true;
            paymentEnt.Exec(1, "SetRowAndSeat", SeatInformationArray[seat][1], SeatInformationArray[seat][2]);
            seat_found = true;
            break;
        }
        
		
    }
    
    if (seat_found)
    {
        SetEmptyPlaza();
        ProceedForward = true;
    }
}

function CancelClicked()
{
    SetEmptyPlaza();
    ProceedBackward = true;
}

function Update()
{
    if (ProceedBackward)
    {
        var loginPos = loginPlaceable.Position();
        var seatPos = seatPlaceable.Position();
        var paymentPos = paymentPlaceable.Position();

        loginPos.z += 0.5;
        loginPos.y -= 0.2;
        loginPlaceable.SetPosition(loginPos);
        seatPlaceable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / rotation));
        
        seatPos.y -= 0.2;
        seatPos.z -= 0.2;
        seatPlaceable.SetPosition(seatPos); 

        paymentPos.y -= 0.2;
        paymentPlaceable.SetPosition(paymentPos);

        rotation += 0.01;
        
        if (rotation >= 1.2)       
        {
            ProceedBackward = false;
            seatPlaceable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / 1.2));
            rotation = 1.0;
        }
        return;
    }

    if (ProceedForward)
    {
        var seatPos = seatPlaceable.Position();
        var paymentPos = paymentPlaceable.Position();
        
        paymentPos.z += 0.2;
        paymentPos.y += 0.2;
        paymentPlaceable.SetPosition(paymentPos);
        paymentPlaceable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / rotation_2));

        seatPos.z -= 0.49;
        seatPos.y += 0.13;
        seatPlaceable.SetPosition(seatPos);        
        rotation_2 -= 0.01;

        if (rotation_2 <= 1.0)       
        {
            ProceedForward = false;
            paymentPlaceable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / 1.0));
            rotation_2 = 1.2;
        }
        return;
    }
}

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (/*...*/true)
    {
        //...
        return;
    }
}
