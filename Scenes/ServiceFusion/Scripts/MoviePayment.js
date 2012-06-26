engine.IncludeFile("MovieStyleSheets.js");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");

me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);
frame.Updated.connect(Update);

var calendarEntity = scene.GetEntityByName("calendar");
var calendar_component = calendarEntity.GetComponent("EC_VisualContainer", "calendar");

if (!server.IsRunning() && !framework.IsHeadless())
{
    engine.ImportExtension("qt.core");
    engine.ImportExtension("qt.gui");
}

var payment_container = new BaseContainer(null);
// todo replace this with layout object.
payment_container.visual.setLayout(new QHBoxLayout());
payment_container.visual.layout().setContentsMargins(0, 0, 0, 0);
payment_container.visual.layout().setSpacing(0);

var credit_card_container = new BaseContainer(payment_container);
var credit_script = new Script();
credit_script.Invoked.connect(CreditScript);
payment_container.visual.owner.eventManager.RegisterScript(new Tag(RdfVocabulary.sourceApplication, "CreditCard"), credit_script);
 
function CreditScript(tag, rdfStore)
{
    if (tag.data == "CreditCard")
    {
        credit_card_container.container.rdfStore.FromString(rdfStore.toString());
        
        var variables = new Array();
        var statements = Select(credit_card_container.container.rdfStore, null, RdfVocabulary.data, null);
        if (statements.length == 3)
        {
            for (var i = 0; i < statements.length; ++i)
            {
                variables.push(statements[i].object.literal);
            }
        }
        ReleaseStatements(statements);
        
        //DisplayCreditCardInfo(variables);
        CardReceived(variables);
    }
}

function DisplayCreditCardInfo(varibles)
{
    if (varibles.length == 3)
    {
        var label_paymentarea = findChild(payment_container.visual, "label_paymentarea");
        label_paymentarea.text = "\n\n" + varibles[0] + ": " + varibles[1] + "\n\n";
    }
}

var visa_entity = scene.GetEntityByName("visa_card");

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

var frame_payment = new QFrame();
var frame_payment_2 = new QFrame();
var frame_thankyou = new QFrame();

me.Action("SetRowAndSeat").Triggered.connect(SetRowAndSeatNumber);
me.Action("SetMovieInfo").Triggered.connect(SetMovieInfo);
me.Action("CardReceived").Triggered.connect(CardReceived);

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
var loginEnt = scene.EntityByName("MovieLoginDialog");
var paymentPlaceable = paymentEnt.GetOrCreateComponent("EC_Placeable");
var seatPlaceable = seatEnt.GetOrCreateComponent("EC_Placeable");

var movieName = "Elokuvan nimi";
var moviePlace = "Salinumero";
var movieTime = "Aika";
var movieDate = "Päivämäärä";
var seatNumber = 0;
var rowNumber = 0;

var frame_2_created = false;

var label_seat = new QLabel("Rivi: " + rowNumber + " Paikka: " + seatNumber);

var label_movieinfo = new QLabel(movieName + " - " + moviePlace + " - " + movieTime + " - " + movieDate);
label_movieinfo.wordWrap = true;

var label_seat2 = new QLabel("Rivi: " + rowNumber + " Paikka: " + seatNumber);
var label_movieinfo2 = new QLabel(movieName + " - " + moviePlace + " - " + movieTime + " - " + movieDate);
label_seat.setStyleSheet(NormalText);
label_movieinfo.setStyleSheet(NormalText);
label_seat2.setStyleSheet(NormalText);
label_movieinfo2.setStyleSheet(NormalText);
StartPayment();

function StartPayment()
{
    var buttonCancel = new QPushButton("Peruuta");
    buttonCancel.clicked.connect(CancelClicked);
    buttonCancel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);



    var label_paymentarea = new QLabel("\n\n\nRaahaa maksukortti tälle alueelle\n\n\n");
    label_paymentarea.setStyleSheet(PaymentArea);
    label_paymentarea.objectName = "label_paymentarea";
    label_paymentarea.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
    

    var label_shoppingcart = new QLabel("Tuote:");
    var label_price = new QLabel("Summa: 9.00 euroa");

    buttonCancel.setStyleSheet(NormalButton);
    label_shoppingcart.setStyleSheet(LargeBoldText);
    label_movieinfo.setStyleSheet(LargeText);
    label_price.setStyleSheet(Price);

    var grid = new QGridLayout();
    grid.setVerticalSpacing(8);

    grid.addWidget(label_shoppingcart, 0, 0, 1, 1);
    grid.addWidget(label_movieinfo, 0, 1, 1, 1);

    grid.addWidget(label_seat, 1, 0, 1, 1);
    grid.addWidget(label_price, 1, 1, 1, 1);
    
    grid.addWidget(label_paymentarea, 2, 0, 1 , 2);

    var buttonLayout = new QHBoxLayout();
    buttonLayout.addWidget(buttonCancel, 0, 0);
    buttonLayout.setContentsMargins(0,0,0,0);

    var vertLayout = new QVBoxLayout();
    vertLayout.addLayout(grid);
    vertLayout.addSpacerItem(new QSpacerItem(1,1, QSizePolicy.Fixed, QSizePolicy.Expanding));
    vertLayout.addLayout(buttonLayout);
    vertLayout.setContentsMargins(50, 50, 50, 50);

    frame_payment.setLayout(vertLayout);

    payment_container.visual.layout().addWidget(frame_payment, 0, 0);
    me.graphicsviewcanvas.GraphicsScene().addWidget(payment_container.visual);
    me.graphicsviewcanvas.width = frame_payment.width;
    me.graphicsviewcanvas.height = frame_payment.height;
    frame_payment_2.hide();
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

function CardReceived(variables)
{
    if (!frame_2_created)
    {
        var buttonOK = new QPushButton("Maksa");
        buttonOK.clicked.connect(OKClicked);
        var buttonCancel = new QPushButton("Peruuta");
        buttonCancel.clicked.connect(Cancel2Clicked);

        buttonOK.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
        buttonCancel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
        
        var le_username = new QLineEdit();
        le_username.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
        var le_password = new QLineEdit();
        le_password.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

        var label_shoppingcart = new QLabel("Tuote:");
        
        
        var label_price = new QLabel("Summa: 9.00 euroa");

        label_shoppingcart.setStyleSheet(LargeBoldText);
        label_movieinfo.setStyleSheet(LargeText);

        label_price.setStyleSheet(Price);
        
        var label_transfer = new QLabel("\nVerkkomaksu");
        label_transfer.setStyleSheet(LargeBoldText);

        var label_info = new QLabel("Anna käyttäjätunnuksesi ja salasanasi alla\noleviin kenttiin ja paina Maksa-painiketta.\n");

        var label_username = new QLabel("Käyttäjätunnus:");
        var label_password = new QLabel("Salasana:");

        buttonOK.setStyleSheet(NormalButton);
        buttonCancel.setStyleSheet(NormalButton);
        label_shoppingcart.setStyleSheet(LargeBoldText);
        label_price.setStyleSheet(Price);
        label_transfer.setStyleSheet(LargeBoldText);
        label_info.setStyleSheet(LargeText);
        label_username.setStyleSheet(LargeText);
        label_password.setStyleSheet(LargeText);
	

        var grid = new QGridLayout();
        grid.setVerticalSpacing(8);

        grid.addWidget(label_shoppingcart, 0, 0, 1, 1);
        grid.addWidget(label_movieinfo2, 0, 1, 1, 1);

        grid.addWidget(label_seat2, 1, 0, 1, 1);
        grid.addWidget(label_price, 1, 1, 1, 1);
        
        grid.addWidget(label_transfer, 2, 0, 1, 2);
        grid.addWidget(label_info, 3, 0, 1, 2);
        
        grid.addWidget(label_username, 4, 0);
        grid.addWidget(le_username, 4, 1, Qt.AlignLeft, 2);
        
        grid.addWidget(label_password, 5, 0);
        grid.addWidget(le_password, 5, 1, Qt.AlignLeft, 2);

        var buttonLayout = new QHBoxLayout();
        buttonLayout.addWidget(buttonCancel, 0, 0);
        buttonLayout.addWidget(buttonOK, 0, 0);
        buttonLayout.setContentsMargins(0,50,0,0);

        var vertLayout = new QVBoxLayout();
        vertLayout.addLayout(grid);
        vertLayout.addSpacerItem(new QSpacerItem(1,1, QSizePolicy.Fixed, QSizePolicy.Expanding));
        vertLayout.addLayout(buttonLayout);
        vertLayout.setContentsMargins(50, 50, 50, 50);

        frame_payment_2.setLayout(vertLayout);
        me.graphicsviewcanvas.GraphicsScene().addWidget(frame_payment_2);
    }
    placeable.SetScale(1.0, 0.5, 1);
    frame_payment.hide();   
    me.graphicsviewcanvas.width = frame_payment_2.width;
    me.graphicsviewcanvas.height = frame_payment_2.height;
    frame_payment_2.show();

    le_username.setText(variables[0]);
    le_password.setText(variables[1]);
}

function SetRowAndSeatNumber(row, seat)
{
	if (visa_entity)
		visa_entity.placeable.visible = true;
		
    rowNumber = row;
    seatNumber = seat;
    label_seat.setText("Rivi: " + rowNumber + " Paikka: " + seatNumber);
    label_seat2.setText("Rivi: " + rowNumber + " Paikka: " + seatNumber);
}

function OKClicked()
{
	if (visa_entity)
		visa_entity.placeable.visible = false;
    ThankYou();
}

function CancelClicked()
{
	if (visa_entity)
		visa_entity.placeable.visible = false;
    ProceedBackward = true;
}

function Cancel2Clicked()
{
    frame_payment_2.hide();
    me.graphicsviewcanvas.width = frame_payment.width;
    me.graphicsviewcanvas.height = frame_payment.height;
    frame_payment.show();
    placeable.SetScale(1.0, 0.5, 1);
}

function SetMovieInfo(name, place, time, date)
{
    movieName = name;
    moviePlace = place;
    movieTime = time;
    movieDate = date;
    label_movieinfo.setText(movieName + " - " + moviePlace + " - " + movieTime + " - " + movieDate);
    label_movieinfo2.setText(movieName + " - " + moviePlace + " - " + movieTime + " - " + movieDate);
}

function ThankYou()
{
    var buttonOK = new QPushButton("OK");
    buttonOK.setStyleSheet(NormalButton);
    buttonOK.clicked.connect(ThankyouClicked);

    buttonOK.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
    
    var label_thankyou = new QLabel("Kiitos lippuostoksestasi,\ntervetuloa elokuviin!\n");
    var label_moviename = new QLabel(movieName);
    var label_movietime = new QLabel(movieTime);
    var label_movieplace = new QLabel(moviePlace);
    var label_moviedate = new QLabel(movieDate.toString());
    var label_row = new QLabel("Rivi: " + rowNumber);
    var label_seat = new QLabel("Paikka: " + seatNumber + "\n");

    label_thankyou.setStyleSheet(LargeText);
    label_moviename.setStyleSheet(NormalText);
    label_movieplace.setStyleSheet(NormalText);
    label_row.setStyleSheet(NormalText);
    label_seat.setStyleSheet(NormalText);

    label_movietime.setStyleSheet(NormalTextRightBold);
    label_moviedate.setStyleSheet(NormalTextRight);
    
    var grid = new QGridLayout();
    grid.setVerticalSpacing(8);

    grid.addWidget(label_thankyou, 0, 0, 1, 2);
    grid.addWidget(label_moviename, 1, 0, 1, 1);
    grid.addWidget(label_movietime, 1, 1, 1, 1);
    grid.addWidget(label_movieplace, 2, 0, 1, 1);
    grid.addWidget(label_moviedate, 2, 1, 1, 1);
    grid.addWidget(label_row, 3, 0, 1, 1);
    grid.addWidget(label_seat, 4, 0, 1, 1);     


    var buttonLayout = new QHBoxLayout();
    buttonLayout.addWidget(buttonOK, 0, 0);
    buttonLayout.setContentsMargins(0,0,0,0);

    var vertLayout = new QVBoxLayout();
    vertLayout.addLayout(grid);
    vertLayout.addSpacerItem(new QSpacerItem(1,1, QSizePolicy.Fixed, QSizePolicy.Expanding));
    vertLayout.addLayout(buttonLayout);
    vertLayout.setContentsMargins(80, 50, 80, 50);

    placeable.SetScale(1.0, 0.5, 1);
    
    frame_thankyou.setLayout(vertLayout);
    me.graphicsviewcanvas.GraphicsScene().addWidget(frame_thankyou);
    frame_payment_2.hide();   
    me.graphicsviewcanvas.width = frame_thankyou.width;
    me.graphicsviewcanvas.height = frame_thankyou.height;
    frame_thankyou.show();
}

function ThankyouClicked()
{
    //Activate calendar event and close dialogs etc.
    var movieContainer = new BaseContainer(null);
    AddStatement(movieContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.sourceApplication, "Movie");
    var time = movieTime.split(":");
    var date = movieDate[0].split(".");
    var movie_date = new Date();
    movie_date.setFullYear(parseInt(date[2]), (parseInt(date[1])-1), parseInt(date[0]));
    movie_date.setHours(parseInt(time[0]));
    movie_date.setMinutes(parseInt(time[1]));
    AddStatement(movieContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, movie_date.toString());
    AddStatement(movieContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, movieName);
    AddStatement(movieContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, moviePlace);
    calendar_component.vc.HandleMeshDrop(movieContainer.visual);
    movieContainer.visual.deleteLater();
    
    
    //Create a 3D-ticket
    if (!scene.EntityByName("MovieTicket"))
    {
        // todo remove movie rdf data from the cart container.
        if (scene.EntityByName("cart_item"))
            scene.EntityByName("cart_item").placeable.visible = false;
        if (scene.EntityByName("MovieLoginDialog"))
            scene.RemoveEntity(scene.EntityByName("MovieLoginDialog").id);
        if (scene.EntityByName("MovieSeatDialog"))
            scene.RemoveEntity(scene.EntityByName("MovieSeatDialog").id);
        me.placeable.visible = false;
        var movieTicketEntity = scene.CreateEntity(scene.NextFreeId(), ["EC_Script", "EC_Name"]);
        movieTicketEntity.SetName("MovieTicket");
        var script = movieTicketEntity.GetOrCreateComponent("EC_Script");
        script.scriptRef = new AssetReference("MovieTicket.js");
        script.runOnLoad = true;
        
        script = movieTicketEntity.GetOrCreateComponent("EC_Script", "Screen");
        script.scriptRef = new AssetReference("Screen.js"); 
        script.runOnLoad = true; 
        frame.DelayedExecute(1.0).Triggered.connect(SendTicketData);
    }
}

function SendTicketData()
{
    var movieTicketEntity = scene.EntityByName("MovieTicket");
    movieTicketEntity.Exec(1, "SetTicketInfo", movieName, movieTime, movieDate);
    movieTicketEntity.Exec(1, "SetTicketInfo2", moviePlace, seatNumber, rowNumber);

}

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (payment_container && payment_container.visual)
        payment_container.visual.deleteLater();
}


