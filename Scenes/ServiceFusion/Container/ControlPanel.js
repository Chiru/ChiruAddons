// Make this script asset depend on the ui file so that Tundra automatically loads the ui file when loading this script.
// !ref: ControlPanel.ui
// !ref: explosion-01.wav

me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

// The QtUiAsset::Instantiate function refers to the type 'QWidget'. To be able to call that function,
// we need to import that type into this script. QWidget resides in the qt.gui module.
engine.ImportExtension("qt.gui");

// Instantiate the window. The window will be emdedded to the main window as a root widget (no parent).
var uiWidget = asset.GetAsset("ControlPanel.ui").Instantiate(false, 0);

//print(me.graphicsviewcanvas.GraphicsView());
//uiWidget.setParent(me.graphicsviewcanvas.GraphicsView());
var proxy = me.graphicsviewcanvas.GraphicsScene().addWidget(uiWidget);
// The returned window will be hidden by default. We need to call 'show()' to display it.
// For more functions that QWidget has, see http://doc.qt.nokia.com/latest/qwidget.html .
uiWidget.show();

findChild(uiWidget, "radioFishes").pressed.connect(OnRadioFishesPressed);
function OnRadioFishesPressed()
{
//    me.dynamiccomponent.SetAttribute("spawningHammers", false);
}

findChild(uiWidget, "radioHammers").pressed.connect(OnRadioHammersPressed);
function OnRadioHammersPressed()
{
//    me.dynamiccomponent.SetAttribute("spawningHammers", true);
}

findChild(uiWidget, "pushButton").pressed.connect(OnPushButtonPressed);
function OnPushButtonPressed()
{
    var entities = scene.Entities();
    for(i in entities)
    {
        if (entities[i].name == "Fish" || entities[i].name == "Hammer")
        {
            var pos = entities[i].placeable.WorldPosition();
            if (pos.Length() < 30) // Near world center?
            {
                entities[i].rigidbody.ApplyImpulse(pos.ScaledToLength(50 + Math.random() * 20).Add(new float3(0,100 + Math.random() * 50,0)));
                entities[i].rigidbody.ApplyTorqueImpulse(Math.random() * 10 - 5, Math.random() * 10 - 5, Math.random() * 10 - 5);
            }
        }
    }
    audio.PlaySound(asset.GetAsset("explosion-01.wav"));
}

// We need to explicitly remember to delete the widget when this script closes, or otherwise the window
// will remain on screen even if this script is deleted.
function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (uiWidget)
        uiWidget.deleteLater();
}
