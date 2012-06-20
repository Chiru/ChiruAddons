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
placeable.SetPosition(0, 0, -8.0);
var mesh = me.GetOrCreateComponent("EC_Mesh");
var canvas = me.GetOrCreateComponent("EC_GraphicsViewCanvas");
canvas.submesh = 1;


mesh.meshRef = new AssetReference("news_tablet.mesh");
var materials = mesh.meshMaterial;
materials = ["news_tablet.material", "screen.material"];
mesh.meshMaterial = materials;

var ProceedForward = false;

var loginEnt = scene.EntityByName("MovieLoginDialog");
var seatEnt = scene.EntityByName("MovieSeatDialog");
var loginPlaceable = loginEnt.GetOrCreateComponent("EC_Placeable");
var seatPlaceable = seatEnt.GetOrCreateComponent("EC_Placeable");

var rotation = 1.2;

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

StartLogin();

function StartLogin()
{

    var frame_login = new QFrame();
    

    var buttonOK = new QPushButton("OK");
    buttonOK.clicked.connect(OKClicked);
    var buttonCancel = new QPushButton("Peruuta");
    buttonCancel.clicked.connect(CancelClicked);

    buttonOK.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
    buttonCancel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var le_firstname = new QLineEdit();
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
    
    var rb_female = new QRadioButton("Nainen");


    var label_firstname = new QLabel("Etunimi:");
    var label_lastname = new QLabel("Sukunimi:");
    var label_birthday = new QLabel("Syntymäpäivä (pp/kk/vvvv):");
    var label_gender = new QLabel("Sukupuoli:");
    var label_email = new QLabel("Sähköpostiosoite:");
    var label_phone = new QLabel("Matkapuhelinnumero:");

    var grid = new QGridLayout();
    grid.setVerticalSpacing(8);
    
    grid.addWidget(label_firstname, 0, 0);
    grid.addWidget(le_firstname, 0, 1, Qt.AlignLeft, 2);

    grid.addWidget(label_lastname, 1, 0);
    grid.addWidget(le_lastname, 1, 1, Qt.AlignLeft, 2);

    grid.addWidget(label_birthday, 2, 0);
    grid.addWidget(le_birthday, 2, 1, Qt.AlignLeft, 2);

    grid.addWidget(label_gender, 3, 0);
    grid.addWidget(rb_male, 3, 1);
    grid.addWidget(rb_female, 3, 2);

    grid.addWidget(label_email, 4, 0);
    grid.addWidget(le_email, 4, 1, Qt.AlignLeft, 2);

    grid.addWidget(label_phone, 5, 0);
    grid.addWidget(le_phone, 5, 1, Qt.AlignLeft, 2);
    

    var buttonLayout = new QHBoxLayout();
    buttonLayout.addWidget(buttonCancel, 0, 0);
    buttonLayout.addWidget(buttonOK, 0, 0);

    var vertLayout = new QVBoxLayout();
    vertLayout.addLayout(grid);
    vertLayout.addSpacerItem(new QSpacerItem(1,1, QSizePolicy.Fixed, QSizePolicy.Expanding));
    vertLayout.addLayout(buttonLayout);

    frame_login.setLayout(vertLayout);

    me.graphicsviewcanvas.GraphicsScene().addWidget(frame_login);
    me.graphicsviewcanvas.width = frame_login.width;
    me.graphicsviewcanvas.height = frame_login.height;
    frame_login.show();
    
    
}

function OKClicked()
{
    ProceedForward = true;  
}

function CancelClicked()
{

}

function Update()
{   
    if (ProceedForward)
    {
        //me.graphicsviewcanvas.update = false;
        //seatEnt.graphicsviewcanvas.update = false;
        var loginPos = loginPlaceable.Position();
        var seatPos = seatPlaceable.Position();
        loginPos.z -= 0.1;
        loginPos.y += 0.1;
        loginPlaceable.SetPosition(loginPos);
        seatPlaceable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / rotation));
        
        seatPos.y += 0.2;
        seatPos.z += 0.2;
        seatPlaceable.SetPosition(seatPos); 

        rotation -= 0.01;
        
        if (rotation <= 1.0)       
        {
            ProceedForward = false;
            //me.graphicsviewcanvas.update = true;
            //seatEnt.graphicsviewcanvas.update = true;
            seatPlaceable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / 1.0));
            rotation = 1.2;
        }
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
