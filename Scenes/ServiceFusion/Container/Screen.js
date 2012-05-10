// Screen.js - Creates unique texture name for the GraphicsViewCanvas' output texture

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
