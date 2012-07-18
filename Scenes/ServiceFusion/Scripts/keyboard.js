// !ref: Keyboard.ui

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("CalendarCellWidget.js");
engine.IncludeFile("VisualContainerUtils.js");
engine.IncludeFile("RdfVocabulary.js");

me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

var keyboard_container = me.GetOrCreateComponent("EC_VisualContainer", "keyboard");
var placeable = me.GetOrCreateComponent("EC_Placeable");
placeable.SetParent(scene.EntityByName("UiCamera"), 0);
placeable.SetScale(1.40, 0.65, 1);
placeable.SetPosition(5.5, 0, -11.0);
placeable.transform.rot = float3(0,0,0);
var mesh = me.GetOrCreateComponent("EC_Mesh");
var canvas = me.GetOrCreateComponent("EC_GraphicsViewCanvas");
canvas.submesh = 1;

mesh.meshRef = new AssetReference("news_tablet.mesh");
var materials = mesh.meshMaterial;
materials = ["news_tablet.material", "screen.material"];
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

var keyboardContainer = null;

keyboardContainer = new BaseContainer(null);
keyboard_container.vc = keyboardContainer.visual;

var keyLayout = new QHBoxLayout();
keyLayout.setSpacing(0);
keyLayout.setContentsMargins(0, 0, 0, 0);
keyboardContainer.visual.setLayout(keyLayout);

var keyboardWidget = asset.GetAsset("Keyboard.ui").Instantiate(false, 0);

keyboardWidget.visible = true;
keyboardContainer.visual.size = keyboardWidget.size;
keyLayout.addWidget(keyboardWidget, 0, 0);

me.graphicsviewcanvas.width = keyboardContainer.visual.width;
me.graphicsviewcanvas.height = keyboardContainer.visual.height;
me.graphicsviewcanvas.GraphicsScene().addWidget(keyboardContainer.visual);
keyboardContainer.visual.show();

function KeyboardHandler(visualContainer)
{
    this.paymentEntity = scene.GetEntityByName("MoviePaymentDialog");
    this.buttons = findChildren(visualContainer, "button_.*");
    for(i = 0; i < this.buttons.length; i++)
    {
        this.ConnectButton(this.buttons[i]);
    }
}

KeyboardHandler.prototype.ConnectButton = function(button)
{
    button.pressed.connect(this, function() { this.paymentEntity.Exec(1, "KeyPressed", button.text); });
}

var keyboardHandler = new KeyboardHandler(keyboardContainer.visual);

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (keyboardWidget)
    {
        keyboardWidget.deleteLater();
        keyboardWidget = null;

        keyboardContainer.visual.deleteLater();
        keyboardContainer = null;
    }
}
