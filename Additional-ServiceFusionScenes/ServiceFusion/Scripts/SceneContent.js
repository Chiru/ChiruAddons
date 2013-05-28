SCENENAME = "";
SAVED_URLS = {"Nokia" : "http://www.nokia.com/global", "Intel" : "http://www.intel.com/content/www/us/en/homepage.html"}
SCENE2D = "false";

var storage = framework.CommandLineParameters("--storage");
var config = framework.CommandLineParameters("--config");

if(storage[0].search("office2")>0)
{
    if(config[0].search("ServiceFusionPlugins2D")>0)
    {
        SCENENAME = "office2_flat2D.txml";
        SCENE2D = "true";
    }
    else    
        SCENENAME = "office2.txml";
}
else if(storage[0].search("clubhouse")>0)
    SCENENAME = "clubhouse.txml";

if(SCENENAME == "clubhouse.txml")
{
    NEWS_URL_FIN = "http://www.rumba.fi";
    NEWS_URL_ENG = "http://www.rollingstone.com";
    PRESET_EVENT1_TIME = [2012, 6, 25, 12, 30]; //year, day, month, hour, minute 
    PRESET_EVENT2_TIME = [2012, 6, 25, 22, 30]; //year, day, month, hour, minute
    PRESET_EVENT1_ENG = "Lunch at Rosso";
    PRESET_EVENT2_ENG = "Gig at 45special";
    PRESET_EVENT1_FIN = "Lounas Rossossa";
    PRESET_EVENT2_FIN = "Keikka 45specialissa";
    PRESET_EVENT_SEARCH = "content";
    CAMERA_MIN_X = -410, CAMERA_MAX_X = 440;
    CAMERA_MIN_Z = -410, CAMERA_MAX_Z = 260;
    var uiCamTransPos = scene.EntityByName("UiCamera").placeable.transform.pos;
    uiCamTransPos.x = 169.90;
    uiCamTransPos.y = 208.50;
    scene.EntityByName("UiCamera").placeable.SetPosition(uiCamTransPos);
}
else if(SCENENAME == "office2.txml" || SCENENAME == "office2_flat2D.txml")
{
    NEWS_URL_FIN = "http://www.taloussanomat.fi/sivu.php?page_id=1";
    //NEWS_URL_ENG = "http://europe.wsj.com/home-page";
    NEWS_URL_ENG = "http://m.kaleva.fi";
    PRESET_EVENT1_TIME = [2012, 6, 25, 9, 30]; //year, day, month, hour, minute 
    PRESET_EVENT2_TIME = [2012, 6, 25, 15, 00]; //year, day, month, hour, minute
    PRESET_EVENT1_ENG = "Visit to Intel";
    PRESET_EVENT2_ENG = "Seminar at Nokia";
    PRESET_EVENT1_FIN = "Vierailu Intelillä";
    PRESET_EVENT2_FIN = "Seminaari Nokialla";
    PRESET_EVENT_SEARCH = "user";
    CAMERA_MIN_X = -215, CAMERA_MAX_X = 235;
    CAMERA_MIN_Z = -440, CAMERA_MAX_Z = 140;
    var uiCamTransPos = scene.EntityByName("UiCamera").placeable.transform.pos;
    uiCamTransPos.x = -3.59;
    uiCamTransPos.y = 259.50;
    uiCamTransPos.z = -245.56;
    scene.EntityByName("UiCamera").placeable.SetPosition(uiCamTransPos);
}
else
{
    //default values.
    NEWS_URL = "http://m.kaleva.fi"; 
    NEWS_URL_ENG = "http://www.kaleva.fi/news";
    PRESET_EVENT1_TIME = [2012, 6, 25, 8, 30]; 
    PRESET_EVENT2_TIME = [2012, 6, 25, 14, 30]; 
    PRESET_EVENT1_ENG = "Gym at Raatti";
    PRESET_EVENT2_ENG = "Coffee with Maija at Antell";
    PRESET_EVENT1_FIN = "Salille Raatti";
    PRESET_EVENT2_FIN = "Kahville Maijan kanssa Antellille";
    CAMERA_MIN_X = -500, CAMERA_MAX_X = 500;
    CAMERA_MIN_Z = -500, CAMERA_MAX_Z = 500;
}
