// Icon.js - Script skeleton for 3D icons that want to perform something when clicked.

/*  Currently SceneInteract performs the following entity-actions:
    - "MouseHoverIn"
    - "MouseHover"
    - "MouseHoverOut"
    - "MousePress" parameters: (int)"Qt::MouseButton", (float,float,float)"x,y,z", (int)"submesh index"</div>
    - "MouseRelease" parameters: (int)"Qt::MouseButton", (float,float,float)"x,y,z", (int)"submesh index"</div> */

engine.IncludeFile("Log.js");

me.Action("MousePress").Triggered.connect(OnMousePress);
//me.Action("MouseHover").Triggered.connect(OnMouseHover);

function OnMousePress()
{
    Log("Hello, my name is " + me.name);
}

function OnMouseHover()
{
    Log("Hovering above " + me.name);
}
