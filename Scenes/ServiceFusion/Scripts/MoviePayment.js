engine.IncludeFile("MovieStyleSheets.js");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("VisualContainerUtils.js");
engine.IncludeFile("Localisation.js");

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

var base_widget = new QWidget();
base_widget.setLayout(new QHBoxLayout());
base_widget.layout().setContentsMargins(0, 0, 0, 0);
base_widget.layout().setSpacing(0);
base_widget.size = new QSize(800,640);


var frame_payment = new QFrame();
payment_container.visual.layout().addWidget(frame_payment, 0, 0);
base_widget.layout().addWidget(payment_container.visual, 0, 0);
frame_payment.hide();
var frame_payment_2 = new QFrame();
base_widget.layout().addWidget(frame_payment_2, 0, 0);
frame_payment_2.hide();
var frame_thankyou = new QFrame();
base_widget.layout().addWidget(frame_thankyou, 0, 0);
frame_thankyou.hide();

me.graphicsviewcanvas.GraphicsScene().addWidget(base_widget);

var focusedEdit = "";

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
var managerEnt = scene.EntityByName("MovieManager");

var movieName = LOC_MOVIEPAY_MOVIENAME;
var moviePlace = LOC_MOVIEPAY_MOVIEPLACE;
var movieTime = LOC_MOVIEPAY_MOVIETIME;
var movieDate = LOC_MOVIEPAY_DATE;
var seatNumber = 0;
var rowNumber = 0;

var frame_2_created = false;
var frame_created = false;

var label_seat = new QLabel(LOC_MOVIEPAY_ROW + ": " + rowNumber + " " + LOC_MOVIEPAY_SEAT + ": " + seatNumber);

var label_movieinfo = new QLabel(movieName + " - " + moviePlace + " - " + movieTime + " - " + movieDate);
label_movieinfo.wordWrap = true;

var label_seat2 = new QLabel(LOC_MOVIEPAY_ROW + ": " + rowNumber + " " + LOC_MOVIEPAY_SEAT + ": " + seatNumber);
var label_movieinfo2 = new QLabel(movieName + " - " + moviePlace + " - " + movieTime + " - " + movieDate);
label_seat.setStyleSheet(NormalText);
label_movieinfo.setStyleSheet(NormalText);
label_seat2.setStyleSheet(NormalText);
label_movieinfo2.setStyleSheet(NormalText);
StartPayment();

function StartPayment()
{
    if (!frame_created)
    {
        var buttonCancel = new QPushButton(LOC_COM_CANCEL);
        buttonCancel.clicked.connect(CancelClicked);
        buttonCancel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

        var label_paymentarea = new QLabel("\n\n\n" + LOC_MOVIEPAY_DRAGCARD + "\n\n\n");
        label_paymentarea.setStyleSheet(PaymentArea);
        label_paymentarea.objectName = "label_paymentarea";
        label_paymentarea.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);


        var label_shoppingcart = new QLabel(LOC_MOVIEPAY_ITEM + ":");
        var label_price = new QLabel(LOC_MOVIEPAY_PRESET_SUM);

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
        frame_created = true;
    }

    //me.graphicsviewcanvas.GraphicsScene().addWidget(payment_container.visual);
    base_widget.size = frame_payment.sizeHint;
    me.graphicsviewcanvas.width = base_widget.width;
    me.graphicsviewcanvas.height = base_widget.height;

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
        var buttonOK = new QPushButton(LOC_MOVIEPAY_PAY);
        buttonOK.clicked.connect(OKClicked);
        var buttonCancel = new QPushButton(LOC_COM_CANCEL);
        buttonCancel.clicked.connect(Cancel2Clicked);

        buttonOK.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
        buttonCancel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

        var le_username = new QPushButton();
        le_username.objectName = "le_username";
        le_username.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
        le_username.setStyleSheet("QPushButton {color: black; background-color: white; border: 1px inset grey; text-align: left; height: 20px; font: 18px; }");
        le_username.clicked.connect(UsernameFocused);
        var le_password = new QPushButton();
        le_password.objectName = "le_password";
        le_password.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
        le_password.setStyleSheet("QPushButton {color: black; background-color: white; border: 1px inset grey; text-align: left; height: 20px; font: 18px; }");
        le_password.clicked.connect(PasswordFocused);

        var label_shoppingcart = new QLabel(LOC_MOVIEPAY_ITEM);


        var label_price = new QLabel(LOC_MOVIEPAY_PRESET_SUM);

        label_shoppingcart.setStyleSheet(LargeBoldText);
        label_movieinfo.setStyleSheet(LargeText);

        label_price.setStyleSheet(Price);

        var label_transfer = new QLabel("\n" + LOC_MOVIEPAY_ONLINEPAYMENT);
        label_transfer.setStyleSheet(LargeBoldText);

        var label_info = new QLabel(LOC_MOVIEPAY_INSERTCREDENTIALS);

        var label_username = new QLabel(LOC_COM_USERNAME + ":");
        var label_password = new QLabel(LOC_COM_PASSWORD + ":");

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
        frame_2_created = true;
        //me.graphicsviewcanvas.GraphicsScene().addWidget(frame_payment_2);


    }
    placeable.SetScale(1.0, 0.5, 1);
    frame_payment.hide();
    base_widget.size = frame_payment_2.sizeHint;
    me.graphicsviewcanvas.width = base_widget.width;
    me.graphicsviewcanvas.height = base_widget.height;
    frame_payment_2.show();

    if(!scene.EntityByName("PaymentKeyboard"))
    {
        var keyboardEntity = scene.CreateEntity(scene.NextFreeId(), ["EC_Script", "EC_Name", "EC_Placeable"]);
        keyboardEntity.SetName("PaymentKeyboard");
        var script = keyboardEntity.GetOrCreateComponent("EC_Script");
        script.scriptRef = new AssetReference("keyboard.js");
        script.runOnLoad = true;
    }

    me.Action("KeyPressed").Triggered.connect(KeyPressed);

    /*var username = findChild(frame_payment_2, "le_username");
    if (username)
        username.setText(variables[0]);
    var password = findChild(frame_payment_2, "le_password");
    if (password)
        password.setText(variables[1]);*/
}

function UsernameFocused()
{
    focusedEdit = "le_username";
}

function PasswordFocused()
{
    focusedEdit = "le_password";
}

function KeyPressed(key)
{
    var lineEdit = findChild(frame_payment_2, focusedEdit);

    var char_patt = RegExp("^[0-9a-ä]$", "i");
    if(key.match(char_patt))
    {
        lineEdit.text = lineEdit.text + key;
    }
    else if(key === "ENTER")
    {
        OKClicked();
    }
    else if(key === "<-")
    {
        lineEdit.text = lineEdit.text.substr(0,lineEdit.text.length-1);
    }
    else
    {
        print("Unknown character pressed!");
    }
}

function SetRowAndSeatNumber(row, seat)
{
    if (visa_entity)
        visa_entity.placeable.visible = true;

    rowNumber = row;
    seatNumber = seat;
    label_seat.setText(LOC_MOVIEPAY_ROW + ": " + rowNumber + " " + LOC_MOVIEPAY_SEAT + ": " + seatNumber);
    label_seat2.setText(LOC_MOVIEPAY_ROW + ": " + rowNumber + " " + LOC_MOVIEPAY_SEAT + ": " + seatNumber);
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
    var keyboardEntity = scene.EntityByName("PaymentKeyboard");
    if(keyboardEntity)
        scene.RemoveEntity(keyboardEntity.id);
    ProceedBackward = true;
}

function Cancel2Clicked()
{
    frame_payment_2.hide();
    base_widget.size = frame_payment.sizeHint;
    me.graphicsviewcanvas.width = base_widget.width;
    me.graphicsviewcanvas.height = base_widget.height;
    frame_payment.show();
    var keyboardEntity = scene.EntityByName("PaymentKeyboard");
    if(keyboardEntity)
        scene.RemoveEntity(keyboardEntity.id);
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
    var keyboardEntity = scene.GetEntityByName("PaymentKeyboard");
    if(keyboardEntity)
        keyboardEntity.placeable.visible = false;

    var buttonOK = new QPushButton("OK");
    buttonOK.setStyleSheet(NormalButton);
    buttonOK.clicked.connect(ThankyouClicked);

    buttonOK.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);

    var label_thankyou = new QLabel("Kiitos lippuostoksestasi,\ntervetuloa elokuviin!\n");
    var label_moviename = new QLabel(movieName);
    var label_movietime = new QLabel(movieTime);
    var label_movieplace = new QLabel(moviePlace);
    var label_moviedate = new QLabel(movieDate.toString());
    var label_row = new QLabel(LOC_MOVIEPAY_ROW + ": " + rowNumber);
    var label_seat = new QLabel(LOC_MOVIEPAY_SEAT + ": " + seatNumber + "\n");

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
    //me.graphicsviewcanvas.GraphicsScene().addWidget(frame_thankyou);
    frame_payment_2.hide();

    base_widget.size = frame_thankyou.minimumSizeHint;
    me.graphicsviewcanvas.width = base_widget.width;
    me.graphicsviewcanvas.height = base_widget.height;

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
    if (!scene.EntityByName("MovieTicket_" + movieName + "_" + movieDate + "_" + moviePlace + "_" + movieTime + "_" + seatNumber + "_" + rowNumber))
    {
        // todo remove movie rdf data from the cart container.
        if (scene.EntityByName("cart_item"))
            scene.EntityByName("cart_item").placeable.visible = false;
        if (scene.EntityByName("MovieLoginDialog"))
        {
            scene.EntityByName("MovieLoginDialog").Exec(1, "Cleanup");
            scene.RemoveEntity(scene.EntityByName("MovieLoginDialog").id);
        }
        if (scene.EntityByName("MovieSeatDialog"))
        {
            scene.EntityByName("MovieSeatDialog").Exec(1, "Cleanup");
            scene.RemoveEntity(scene.EntityByName("MovieSeatDialog").id);
        }
        //me.placeable.visible = false;
        var movieTicketEntity = scene.CreateEntity(scene.NextFreeId(), ["EC_Script", "EC_Name"]);
        movieTicketEntity.SetName("MovieTicket_" + movieName + "_" + movieDate + "_" + moviePlace + "_" + movieTime + "_" + seatNumber + "_" + rowNumber);
        movieTicketEntity.GetOrCreateComponent("EC_DynamicComponent", "UserItem");
        var script = movieTicketEntity.GetOrCreateComponent("EC_Script");
        script.scriptRef = new AssetReference("MovieTicket.js");
        script.runOnLoad = true;

        script = movieTicketEntity.GetOrCreateComponent("EC_Script", "Screen");
        script.scriptRef = new AssetReference("Screen.js");
        script.runOnLoad = true;
        frame.DelayedExecute(0.1).Triggered.connect(SendTicketData);
    }
    frame.DelayedExecute(0.2).Triggered.connect(destroyPayment);

    var movieDim = scene.GetEntityByName("MovieDim");
    if(movieDim)
        movieDim.Exec(1, "Hide");
}

function SendTicketData()
{
    var movieTicketEntity = scene.EntityByName("MovieTicket_" + movieName + "_" + movieDate + "_" + moviePlace + "_" + movieTime + "_" + seatNumber + "_" + rowNumber);
    movieTicketEntity.Exec(1, "SetTicketInfo", movieName, movieTime, movieDate);
    movieTicketEntity.Exec(1, "SetTicketInfo2", moviePlace, seatNumber, rowNumber);
}

function destroyPayment()
{
    managerEnt.Exec(1, "destroyMoviePayment");
}

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.

    // Remove payment keyboard on entity deletion.
    var keyboardEntity = scene.EntityByName("PaymentKeyboard");
    if(keyboardEntity)
    {
        // Make cleanup first to get proper deletion of objects created.
        keyboardEntity.Exec(1, "Cleanup");
        scene.RemoveEntity(keyboardEntity.id);
    }

    // payment_container is garbage if entity is deleted and cleanup process started.
    // Causes error but not crash. Cleanup should be done before entity deletion.
    if (payment_container && payment_container.visual)
    {
        payment_container.visual.deleteLater();
        payment_container = null;
    }
}
