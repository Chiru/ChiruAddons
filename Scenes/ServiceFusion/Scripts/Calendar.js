// !ref: CalenderWidget.ui
// !ref: CalendarEvents.ui

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("CalendarCellWidget.js");
engine.IncludeFile("VisualContainerUtils.js");
engine.IncludeFile("RdfVocabulary.js");

var defaultTransform = me.placeable.transform;

me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);
me.Action("Reset").Triggered.connect(function()
{
    me.placeable.transform = defaultTransform;
});

// Override dragObjectName from VisualContainerUtils.js.
//dragObjectName = "calendar_day_block";

var eventsEntityName = "calendar_events";
var eventsEntity = null;
var calendarContainer = null;
var calEventsContainer = null;

Date.prototype.getWeek = function()
{
    var janFirst = new Date(this.getFullYear(), 0, 1);
    return Math.ceil((((this - janFirst) / 86400000) + janFirst.getDay()+1)/7);
}

var date = new Date(2012, 6, 25);

var months = ["Tammikuu", "Helmikuu",  "Maaliskuu",
              "Huhtikuu", "Toukokuu",  "Kesäkuu",
              "Heinäkuu", "Elokuu",    "Syyskuu",
              "Lokakuu",  "Marraskuu", "Joulukuu"];
              
var month = months[date.getMonth()];
var year = date.getFullYear();

function OnEventPress(e)
{
    dragObjectName = "calendar_event_block";
    if (eventsEntity)
    {
        if (this.row == 0 && this.column == 0)
        {
            // To prevent double press event when two EC_GraphicsViewCanvases a top eachother. When have time, replace this with better solution.
            frame.DelayedExecute(0.2).Triggered.connect( function() { eventsEntity.placeable.visible = false; });
        }
    }
}

function OnDayPress(e)
{
    dragObjectName = "calendar_day_block";
    ParseDayEvents(this.container);
    if (eventsEntity)
    {
        if (this.row == 5 && this.column == 3 && !eventsEntity.placeable.visible)
            frame.DelayedExecute(0.2).Triggered.connect( function() { eventsEntity.placeable.visible = true; });
        else
            frame.DelayedExecute(0.2).Triggered.connect( function() { eventsEntity.placeable.visible = false; });
    }
}

function OnDayRelease(e)
{
    
}
DayCellEvents = {"MousePress":OnDayPress, "MouseRelease":OnDayRelease};
EventCellEvents = {"MousePress":OnEventPress};


function CellObject(w, row, column, events)
{
    var visual = new VisualContainer(calendarContainer.visual);
    this.container = new Container(visual);
    this.container.parent = calendarContainer.visual.owner;
    this.container.rdfStore = RdfModule.theWorld.CreateStore();
    visual.setLayout(new QHBoxLayout());
    visual.layout().setContentsMargins(0,0,0,0);
    visual.layout().spacing = 0;
    visual.objectName = "vc_" + row + "_" + column;

    var parent = findChild(w, "cell_" + row + "_" + column);
    this.widget = new Cell();
    this.widget.row = row;
    this.widget.column = column;
    this.widget.container = this.container;
    parent.layout().addWidget(visual, 0, 0);
    visual.layout().addWidget(this.widget, 0, 0);
    if (this.widget)
    {
        if (events)
            this.widget.EventRegister = events;
    }
    MakeDraggable(visual, this.widget);
}

calendarContainer = new BaseContainer(null)
var calLayout = new QHBoxLayout();
calLayout.setSpacing(0);
calLayout.setContentsMargins(0, 0, 0, 0);
calendarContainer.visual.setLayout(calLayout);

var calendarWidget = asset.GetAsset("CalenderWidget.ui").Instantiate(false, 0);

calendarWidget.cells = new Array();
// Get every child cell widget from the calendar widget and store them into a double array.
// Note! cells range should be betweeen row[0-6] and column [0-7] only exeption is row[0] where column range is [0-1].
calendarWidget.cells[0] = [new CellObject(calendarWidget, 0, 0), new CellObject(calendarWidget, 0, 1)];
// Iterate each day widget from the original CalendarWidget.ui file.
for (var i = 1; i < 7; ++i)
{
    calendarWidget.cells[i] = [new CellObject(calendarWidget, i, 0),
                               new CellObject(calendarWidget, i, 1, DayCellEvents),
                               new CellObject(calendarWidget, i, 2, DayCellEvents),
                               new CellObject(calendarWidget, i, 3, DayCellEvents),
                               new CellObject(calendarWidget, i, 4, DayCellEvents),
                               new CellObject(calendarWidget, i, 5, DayCellEvents),
                               new CellObject(calendarWidget, i, 6, DayCellEvents),
                               new CellObject(calendarWidget, i, 7, DayCellEvents)];
}

calendarWidget.cells[0][0].widget.layout().addWidget(new QLabel(month), 0, 0);
calendarWidget.cells[0][1].widget.layout().addWidget(new QLabel(year.toString()), 0, 0);

// Fill july data staticly into the calendar.
// Todo replace this with a concreate implementation.
calendarWidget.cells[1][7].widget.layout().addWidget(new QLabel("1"), 0, 0);

calendarWidget.cells[2][1].widget.layout().addWidget(new QLabel("2"), 0, 0);
calendarWidget.cells[2][2].widget.layout().addWidget(new QLabel("3"), 0, 0);
calendarWidget.cells[2][3].widget.layout().addWidget(new QLabel("4"), 0, 0);
calendarWidget.cells[2][4].widget.layout().addWidget(new QLabel("5"), 0, 0);
calendarWidget.cells[2][5].widget.layout().addWidget(new QLabel("6"), 0, 0);
calendarWidget.cells[2][6].widget.layout().addWidget(new QLabel("7"), 0, 0);
calendarWidget.cells[2][7].widget.layout().addWidget(new QLabel("8"), 0, 0);

calendarWidget.cells[3][1].widget.layout().addWidget(new QLabel("9"), 0, 0);
calendarWidget.cells[3][2].widget.layout().addWidget(new QLabel("10"), 0, 0);
calendarWidget.cells[3][3].widget.layout().addWidget(new QLabel("11"), 0, 0);
calendarWidget.cells[3][4].widget.layout().addWidget(new QLabel("12"), 0, 0);
calendarWidget.cells[3][5].widget.layout().addWidget(new QLabel("13"), 0, 0);
calendarWidget.cells[3][6].widget.layout().addWidget(new QLabel("14"), 0, 0);
calendarWidget.cells[3][7].widget.layout().addWidget(new QLabel("15"), 0, 0);

calendarWidget.cells[4][1].widget.layout().addWidget(new QLabel("16"), 0, 0);
calendarWidget.cells[4][2].widget.layout().addWidget(new QLabel("17"), 0, 0);
calendarWidget.cells[4][3].widget.layout().addWidget(new QLabel("18"), 0, 0);
calendarWidget.cells[4][4].widget.layout().addWidget(new QLabel("19"), 0, 0);
calendarWidget.cells[4][5].widget.layout().addWidget(new QLabel("20"), 0, 0);
calendarWidget.cells[4][6].widget.layout().addWidget(new QLabel("21"), 0, 0);
calendarWidget.cells[4][7].widget.layout().addWidget(new QLabel("22"), 0, 0);

calendarWidget.cells[5][1].widget.layout().addWidget(new QLabel("23"), 0, 0);
calendarWidget.cells[5][2].widget.layout().addWidget(new QLabel("24"), 0, 0);
calendarWidget.cells[5][3].widget.layout().addWidget(new QLabel("25"), 0, 0);

calendarWidget.cells[5][3].widget.styleSheet = Cell.DayActiveStyle;
var visual = calendarWidget.cells[5][3].container.visual;
AddStatement(visual, RdfVocabulary.baseUri, RdfVocabulary.sourceApplication, "datetime");

AddStatement(visual, RdfVocabulary.baseUri, RdfVocabulary.data, Date().toString());
AddStatement(visual, RdfVocabulary.baseUri, RdfVocabulary.data, "Consert");
AddStatement(visual, RdfVocabulary.baseUri, RdfVocabulary.data, "KUNTOSALI RAATTI");

var d1 = new Date (),
    d2 = new Date ( d1 );
d2.setMinutes ( d1.getMinutes() + 30 );
AddStatement(visual, RdfVocabulary.baseUri, RdfVocabulary.data, d2);
AddStatement(visual, RdfVocabulary.baseUri, RdfVocabulary.data, "Important");
AddStatement(visual, RdfVocabulary.baseUri, RdfVocabulary.data, "KAHVILLE MAIJAN KANSSA ANTELLI");

d1 = new Date (),
d2 = new Date ( d1 );
d2.setMinutes ( d1.getMinutes() + 60 );
AddStatement(visual, RdfVocabulary.baseUri, RdfVocabulary.data, d2); 
AddStatement(visual, RdfVocabulary.baseUri, RdfVocabulary.data, "Custom");
AddStatement(visual, RdfVocabulary.baseUri, RdfVocabulary.data, "ELOKUVA MEN IN BLACK III PLAZA 1");

calendarWidget.cells[5][4].widget.layout().addWidget(new QLabel("26"), 0, 0);
calendarWidget.cells[5][5].widget.layout().addWidget(new QLabel("27"), 0, 0);
calendarWidget.cells[5][6].widget.layout().addWidget(new QLabel("28"), 0, 0);
calendarWidget.cells[5][7].widget.layout().addWidget(new QLabel("29"), 0, 0);

calendarWidget.cells[6][1].widget.layout().addWidget(new QLabel("30"), 0, 0);
calendarWidget.cells[6][2].widget.layout().addWidget(new QLabel("31"), 0, 0);

calendarContainer.visual.size = calendarWidget.size;
calLayout.addWidget(calendarWidget, 0, 0);

me.graphicsviewcanvas.width = calendarContainer.visual.width;
me.graphicsviewcanvas.height = calendarContainer.visual.height;
me.graphicsviewcanvas.GraphicsScene().addWidget(calendarContainer.visual);
calendarContainer.visual.show();

// Todo Move code below to a separete file.

calEventsContainer = new BaseContainer(null)
var eventLayout = new QHBoxLayout();
eventLayout.setSpacing(0);
eventLayout.setContentsMargins(0, 0, 0, 0);
calEventsContainer.visual.setLayout(eventLayout);

eventsEntity = scene.GetEntityByName(eventsEntityName);
var eventsWidget = asset.GetAsset("CalendarEvents.ui").Instantiate(false, 0);
calEventsContainer.visual.size = eventsWidget.size;

eventsWidget.cells = new Array();
for (var i = 0; i < 4; ++i)
{
    eventsWidget.cells[i] = [new CellObject(eventsWidget, i, 0, EventCellEvents), new CellObject(eventsWidget, i, 1, EventCellEvents)];
} 
eventsWidget.cells[0][0].widget.layout().addWidget(new QLabel("25"), 0, 0);
eventsWidget.cells[0][1].widget.layout().addWidget(new QLabel("TAPAHTUMAT"), 0, 0);

eventLayout.addWidget(eventsWidget, 0, 0);
eventsEntity.graphicsviewcanvas.width = calEventsContainer.visual.width;
eventsEntity.graphicsviewcanvas.height = calEventsContainer.visual.height;
eventsEntity.graphicsviewcanvas.GraphicsScene().addWidget(calEventsContainer.visual);
calEventsContainer.visual.show();

function CalendarEvent(time, name, text)
{
    this.time = time;
    this.name = name;
    this.text = text;
}

var oldEventWidgets = new Array();
function ParseDayEvents(container)
{
    var events = new Array();
    // todo remember to relaese statments.
    var statements = Select(container.rdfStore, null, RdfVocabulary.sourceApplication, "datetime");
    if (statements.length > 0)
    {
        ReleaseStatements(statements);
        statements = Select(container.rdfStore, null, RdfVocabulary.data, null);
        if (statements.length > 0 && (statements.length % 3) == 0)
        {
            for (var i = 0; (i + 3) <= statements.length; i = i +3)
            {
                events.push(new CalendarEvent(new Date(statements[i].object.literal), statements[i+1].object.literal, statements[i+2].object.literal)); 
            }
        }
        ReleaseStatements(statements);
    }
    
    for (var i = 0; i < oldEventWidgets.length; ++i)
        oldEventWidgets[i].deleteLater();
    oldEventWidgets = new Array();
    
    var header = null, text = null;
    for (var i = 0; i < events.length; ++i)
    {
        if (i > 2) break;
        var t = events[i].time;
        var str = t.getHours() + ":" + t.getMinutes();
        header = new QLabel(str);
        text = new QLabel(events[i].text);
        oldEventWidgets.push(header);
        oldEventWidgets.push(text);
        text.wordWrap = true;
        eventsWidget.cells[(i + 1)][0].widget.layout().addWidget(header, 0, 0);
        eventsWidget.cells[(i + 1)][1].widget.layout().addWidget(text, 0, 0);
    }
}

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (calendarWidget)
    {
        calendarWidget.deleteLater();
        calendarWidget = null;
        
        calendarContainer.visual.deleteLater();
        calendarContainer = null;
    }
    if (eventsWidget)
    {
        eventsWidget.deleteLater();
        eventsWidget = null;
        
        calEventsContainer.visual.deleteLater();
        calEventsContainer = null;
    }
}
