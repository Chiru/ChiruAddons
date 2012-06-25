engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");

me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);
frame.Updated.connect(Update);

if (!server.IsRunning() && !framework.IsHeadless())
{
    engine.ImportExtension("qt.core");
    engine.ImportExtension("qt.gui");
}

var id_entity = scene.GetEntityByName("ID_card");
if (id_entity)
	id_entity.placeable.visible = true;
	
var loginContainer = new BaseContainer(null);
var idContainer = new BaseContainer(loginContainer);
var movieContainer = new BaseContainer(loginContainer);
// todo replace this with layout object.
loginContainer.visual.setLayout(new QHBoxLayout());
loginContainer.visual.layout().setContentsMargins(0, 0, 0, 0);
loginContainer.visual.layout().setSpacing(0);
var idScript = new Script();
idScript.Invoked.connect(IdScript);
loginContainer.visual.owner.eventManager.RegisterScript(new Tag(RdfVocabulary.sourceApplication, "ID"), idScript);

function IdScript(tag, rdfStore)
{
    if (tag.data == "ID")
    {
        //\todo For some reason redland rdf lib will mess up statement order, most likely map container is beeing used.
        idContainer.container.rdfStore.FromString(rdfStore.toString());
        
        var variables = new Array();
        var statements = Select(idContainer.container.rdfStore, null, RdfVocabulary.data, null);
        if (statements.length == 6)
        {
            for (var i = 0; i < statements.length; ++i)
            {
                variables.push(statements[i].object.literal);
            }
        }
        ReleaseStatements(statements);
        
        DisplayIdInfo(variables);
    }
}

function DisplayIdInfo(variables)
{
    var le_firstname = findChild(loginContainer.visual, "le_firstname");
    var le_lastname = findChild(loginContainer.visual, "le_lastname");
    /*var le_email = findChild(loginContainer.visual, "le_email");
    var le_phone = findChild(loginContainer.visual, "le_phone");
    var le_birthday = findChild(loginContainer.visual, "le_birthday");
    var rb_gender_male = findChild(loginContainer.visual, "rb_gender_male");
    var rb_gender_female = findChild(loginContainer.visual, "rb_gender_female");*/

    le_firstname.text = variables[0];
    le_lastname.text = variables[1];
    /*le_email.text = variables[2];
    le_phone.text = variables[3];
    le_birthday.text = variables[4];
    if (variables[5] == "Male")
        rb_gender_male.checked = true;
    else
        rb_gender_female.checked = true;*/
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
var paymentEnt = scene.EntityByName("MoviePaymentDialog");
var loginPlaceable = loginEnt.GetOrCreateComponent("EC_Placeable");
var seatPlaceable = seatEnt.GetOrCreateComponent("EC_Placeable");
var paymentPlaceable = paymentEnt.GetOrCreateComponent("EC_Placeable");

var movieName = "Elokuvan nimi";
var moviePlace = "Salinumero";
var movieTime = "Aika";
var movieDate = "Päivämäärä";
var label_movieinfo = new QLabel(movieName + " - " + moviePlace + " - " + movieTime + " - " + movieDate);

me.Action("SetMovieInfo").Triggered.connect(SetMovieInfo);


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
    frame_login.objectName = "frame_login";

    var buttonOK = new QPushButton("Rekisteröidy");
    buttonOK.clicked.connect(OKClicked);
    var buttonCancel = new QPushButton("Peruuta");
    buttonCancel.clicked.connect(CancelClicked);

    buttonOK.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
    buttonCancel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var le_firstname = new QLineEdit();
	le_firstname.objectName = "le_firstname";
    le_firstname.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var le_lastname = new QLineEdit();
	le_lastname.objectName = "le_lastname";
    le_lastname.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var le_email = new QLineEdit();
	le_email.objectName = "le_email";
    le_email.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var le_phone = new QLineEdit();
	le_phone.objectName = "le_phone";
    le_phone.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var le_birthday = new QLineEdit();
	le_birthday.objectName = "le_birthday";
    le_birthday.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var rb_male = new QRadioButton("Mies");
	rb_male.objectName = "rb_gender_male";
    
    var rb_female = new QRadioButton("Nainen");
	rb_male.objectName = "rb_gender_female";

    var label_firstname = new QLabel("Etunimi:");
    var label_lastname = new QLabel("Sukunimi:");
    var label_birthday = new QLabel("Syntymäpäivä (pp/kk/vvvv):");
    var label_gender = new QLabel("Sukupuoli:");
    var label_email = new QLabel("Sähköpostiosoite:");
    var label_phone = new QLabel("Matkapuhelinnumero:");

    var label_shoppingcart = new QLabel("Ostoskori:");
    

    label_shoppingcart.setStyleSheet("QLabel { font: bold 18px; }");
    label_movieinfo.setStyleSheet("QLabel { font: 18px; }");
    label_movieinfo.wordWrap = true;
	

    var grid = new QGridLayout();
    grid.setVerticalSpacing(8);

    grid.addWidget(label_shoppingcart, 0, 0, 1, 3);
    grid.addWidget(label_movieinfo, 1, 0, 1, 3);
    
    grid.addWidget(label_firstname, 2, 0);
    grid.addWidget(le_firstname, 2, 1, Qt.AlignLeft, 2);

    grid.addWidget(label_lastname, 3, 0);
    grid.addWidget(le_lastname, 3, 1, Qt.AlignLeft, 2);

    grid.addWidget(label_birthday, 4, 0);
    grid.addWidget(le_birthday, 4, 1, Qt.AlignLeft, 2);

    grid.addWidget(label_gender, 5, 0);
    grid.addWidget(rb_male, 5, 1);
    grid.addWidget(rb_female, 5, 2);

    grid.addWidget(label_email, 6, 0);
    grid.addWidget(le_email, 6, 1, Qt.AlignLeft, 2);

    grid.addWidget(label_phone, 7, 0);
    grid.addWidget(le_phone, 7, 1, Qt.AlignLeft, 2);
    

    var buttonLayout = new QHBoxLayout();
    buttonLayout.addWidget(buttonCancel, 0, 0);
    buttonLayout.addWidget(buttonOK, 0, 0);

    var vertLayout = new QVBoxLayout();
    vertLayout.addLayout(grid);
    vertLayout.addSpacerItem(new QSpacerItem(1,1, QSizePolicy.Fixed, QSizePolicy.Expanding));
    vertLayout.addLayout(buttonLayout);
    vertLayout.setContentsMargins(20, 20, 20, 20);

    frame_login.setLayout(vertLayout);

    loginContainer.visual.layout().addWidget(frame_login, 0, 0);
    me.graphicsviewcanvas.GraphicsScene().addWidget(loginContainer.visual);
    me.graphicsviewcanvas.width = frame_login.width;
    me.graphicsviewcanvas.height = frame_login.height;
    frame_login.show();
}

function OKClicked()
{
	if (id_entity)
		id_entity.placeable.visible = false;
    ProceedForward = true;
}

function CancelClicked()
{
	if (id_entity)
		id_entity.placeable.visible = false;
    //Destroy all movie entities
}

function Update()
{   
    if (ProceedForward)
    {
        var loginPos = loginPlaceable.Position();
        var seatPos = seatPlaceable.Position();
        var paymentPos = paymentPlaceable.Position();
        loginPos.z -= 0.5;
        loginPos.y += 0.2;
        loginPlaceable.SetPosition(loginPos);
        seatPlaceable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / rotation));
        
        seatPos.y += 0.2;
        seatPos.z += 0.2;
        seatPlaceable.SetPosition(seatPos);

        paymentPos.y += 0.2;
        paymentPlaceable.SetPosition(paymentPos); 

        rotation -= 0.01;
        
        if (rotation <= 1.0)       
        {
            ProceedForward = false;
            seatPlaceable.SetOrientation(Quat(float3(1,0,0), 2*Math.PI / 1.0));
            rotation = 1.2;
            seatEnt.Exec(1, "SeatSelection");
        }
    }   
}

function SetMovieInfo(name, place, time, date)
{
    if (id_entity)
        id_entity.placeable.visible = true;

    movieName = name;
    moviePlace = place;
    movieTime = time;
    movieDate = date;
    label_movieinfo.setText(movieName + " - " + moviePlace + " - " + movieTime + " - " + movieDate);
}

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (loginContainer && loginContainer.visual)
        loginContainer.visual.deleteLater();
    if (idContainer && idContainer.visual)
        idContainer.visual.deleteLater();
    if (movieContainer && movieContainer.visual)
        movieContainer.visual.deleteLater();
        
    if (/*...*/true)
    {
        //...
        return;
    }
}
