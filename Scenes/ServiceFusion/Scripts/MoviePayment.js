me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);
frame.Updated.connect(Update);

if (!server.IsRunning() && !framework.IsHeadless())
{
    engine.ImportExtension("qt.core");
    engine.ImportExtension("qt.gui");
}



var placeable = me.GetOrCreateComponent("EC_Placeable");
placeable.SetParent(scene.EntityByName("UiCamera"), 0);
placeable.SetScale(1, 0.5, 1);
//placeable.SetPosition(0, 0, -8.0);
placeable.SetPosition(0, -8, -12.0);
placeable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / 1.2));
var mesh = me.GetOrCreateComponent("EC_Mesh");
var canvas = me.GetOrCreateComponent("EC_GraphicsViewCanvas");
canvas.submesh = 1;


mesh.meshRef = new AssetReference("news_tablet.mesh");
var materials = mesh.meshMaterial;
materials = ["news_tablet.material", "screen.material"];
mesh.meshMaterial = materials;

var ProceedForward = false;
var ProceedBackward = false;

var rotation = 1.0;

me.Action("SetRowAndSeat").Triggered.connect(SetRowAndSeatNumber);

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

var paymentEnt = scene.EntityByName("MoviePaymentDialog");
var seatEnt = scene.EntityByName("MovieSeatDialog");
var paymentPlaceable = paymentEnt.GetOrCreateComponent("EC_Placeable");
var seatPlaceable = seatEnt.GetOrCreateComponent("EC_Placeable");

var movieName = "Elokuvan nimi";
var moviePlace = "Salinumero";
var movieTime = "Aika";
var movieDate = "Päivämäärä";
var seatNumber = 0;
var rowNumber = 0;

var label_seat = new QLabel("Rivi: " + rowNumber + " Paikka: " + seatNumber);
StartLogin();

function StartLogin()
{

    var frame_payment = new QFrame();
    
    

    var buttonOK = new QPushButton("Seuraava");
    buttonOK.clicked.connect(OKClicked);
    var buttonCancel = new QPushButton("Peruuta");
    buttonCancel.clicked.connect(CancelClicked);

    buttonOK.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
    buttonCancel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    /*var le_firstname = new QLineEdit();
    le_firstname.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var le_lastname = new QLineEdit();
    le_lastname.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var le_email = new QLineEdit();
    le_email.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var le_phone = new QLineEdit();
    le_phone.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var le_birthday = new QLineEdit();
    le_birthday.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var le_email = new QLineEdit();
    le_email.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var le_phone = new QLineEdit();
    le_phone.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var rb_male = new QRadioButton("Mies");
    
    var rb_female = new QRadioButton("Nainen");*/


    var label_paymentarea = new QLabel("\n\nRaahaa maksukortti tälle alueelle\n\n");
    label_paymentarea.setStyleSheet("QLabel { font: bold 18px; qproperty-alignment: AlignCenter; border: 2px solid black;}");
    //label_paymentarea.setFixedSize(300, 200);
    label_paymentarea.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
    

    var label_shoppingcart = new QLabel("Tuote:");
    var label_movieinfo = new QLabel(movieName + " - " + moviePlace + " - " + movieTime + " - " + movieDate);
    
    var label_price = new QLabel("Summa: 9.00 euroa");

    label_shoppingcart.setStyleSheet("QLabel { font: bold 18px; }");
    label_movieinfo.setStyleSheet("QLabel { font: 18px; }");

    label_price.setStyleSheet("QLabel { font: bold 18px; qproperty-alignment: AlignRight; }");
	

    var grid = new QGridLayout();
    grid.setVerticalSpacing(8);

    grid.addWidget(label_shoppingcart, 0, 0, 1, 1);
    grid.addWidget(label_movieinfo, 0, 1, 1, 1);

    grid.addWidget(label_seat, 1, 0, 1, 1);
    grid.addWidget(label_price, 1, 1, 1, 1);
    
    grid.addWidget(label_paymentarea, 2, 0, 1 , 2);

    var buttonLayout = new QHBoxLayout();
    buttonLayout.addWidget(buttonCancel, 0, 0);
    //buttonLayout.addWidget(buttonOK, 0, 0);

    var vertLayout = new QVBoxLayout();
    vertLayout.addLayout(grid);
    vertLayout.addSpacerItem(new QSpacerItem(1,1, QSizePolicy.Fixed, QSizePolicy.Expanding));
    vertLayout.addLayout(buttonLayout);
    vertLayout.setContentsMargins(50, 50, 50, 50);

    frame_payment.setLayout(vertLayout);

    me.graphicsviewcanvas.GraphicsScene().addWidget(frame_payment);
    me.graphicsviewcanvas.width = frame_payment.width;
    me.graphicsviewcanvas.height = frame_payment.height;
    frame_payment.show();
    
    
}

function Update()
{
    if (ProceedBackward)
    {
        var seatPos = seatPlaceable.Position();
        var paymentPos = paymentPlaceable.Position();
        
        paymentPos.z -= 0.2;
        paymentPos.y -= 0.2;
        paymentPlaceable.SetPosition(paymentPos);
        paymentPlaceable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / rotation));

        seatPos.z += 0.49;
        seatPos.y -= 0.13;
        seatPlaceable.SetPosition(seatPos);        
        rotation += 0.01;

        if (rotation >= 1.2)       
        {
            ProceedBackward = false;
            paymentPlaceable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / 1.2));
            rotation = 1.0;
            seatEnt.Exec(1, "SeatSelection");
        }
        return;
    }

}

function SetRowAndSeatNumber(row, seat)
{
    rowNumber = row;
    seatNumber = seat;
    console.LogInfo(row + seat);
    label_seat.setText("Rivi: " + rowNumber + " Paikka: " + seatNumber);
}

function OKClicked()
{
    ProceedForward = true;  
}

function CancelClicked()
{
    ProceedBackward = true;
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


