// !ref: CalenderWidget.ui
// !ref: CalendarEvents.ui

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");
engine.IncludeFile("CalendarCellWidget.js");
engine.IncludeFile("VisualContainerUtils.js");
engine.IncludeFile("RdfVocabulary.js");
engine.IncludeFile("Localisation.js");
engine.IncludeFile("SceneContent.js");

var defaultTransform = me.placeable.transform;

me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);
me.Action("Reset").Triggered.connect(function()
{
    me.placeable.transform = defaultTransform;
});

var calendar_container = me.GetOrCreateComponent("EC_VisualContainer", "calendar");
var event_container = me.GetOrCreateComponent("EC_VisualContainer", "calendar_event");
var day_container = me.GetOrCreateComponent("EC_VisualContainer", "active_day");

var eventsEntityName = "calendar_events";
var eventsEntity = null;
var calendarContainer = null;
var calEventsContainer = null;
var currentDayCell = null;
// Override from VisualContainerUtils.js
dragObjectName = "calendar_day_block";

function AddEventToCalendar(tag, rdfStore)
{
    if (tag.data == "Movie")
    {
        if (!currentDayCell.container)
            return;
        var container = currentDayCell.container;

        var statements = Select(rdfStore, null, RdfVocabulary.data, null);
        if (statements.length >= 2)
        {
            var date = new Date(statements[0].object.literal);
            var movieTitle = statements[1].object.literal;
            var audit = statements[2].object.literal;
            
            var eventContainer = new BaseContainer(container.visual);
            eventContainer.container.parent = container;
            
            AddStatement(eventContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.sourceApplication, "datetime");

            AddStatement(eventContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, date.toString());
            AddStatement(eventContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, "Movie");
            AddStatement(eventContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, movieTitle + "\n" + audit.toUpperCase());
            
            day_container.vc = container.visual;
            ParseDayEvents(container);
            eventsEntity.placeable.visible = true;
        }
    }
}

function DragAddEvent(tag, rdfStore) 
{
    if (tag.data == "Movie")
    {
        var statements = Select(rdfStore, null, RdfVocabulary.data, null);
        if (statements.length >= 2)
        {
            var date = new Date(statements[0].object.literal);
            var movieTitle = statements[1].object.literal;
            var audit = statements[2].object.literal; 
            
            var vc = new VisualContainer(this.container.visual);
            var eventContainer = new Container(vc);
            eventContainer.parent = this.container;
            eventContainer.rdfStore = RdfModule.theWorld.CreateStore();
            
            AddStatement(vc, RdfVocabulary.baseUri, RdfVocabulary.sourceApplication, "datetime");

            AddStatement(vc, RdfVocabulary.baseUri, RdfVocabulary.data, date.toString());
            AddStatement(vc, RdfVocabulary.baseUri, RdfVocabulary.data, "Movie");
            AddStatement(vc, RdfVocabulary.baseUri, RdfVocabulary.data, movieTitle + "\n" + audit.toUpperCase());
            
            day_container.vc = this.container.visual;
            ParseDayEvents(this.container);
            eventsEntity.placeable.visible = true;
        }
        ReleaseStatements(statements);
    }
}

function DragAddEventToParent(tag, rdfStore) 
{
    if (tag.data == "Movie" && day_container.vc)
    {
        var statements = Select(rdfStore, null, RdfVocabulary.data, null);
        if (statements.length >= 2)
        {
            var date = new Date(statements[0].object.literal);
            var movieTitle = statements[1].object.literal; 
            var audit = statements[2].object.literal; 
            
            var vc = new VisualContainer(day_container.vc);
            var eventContainer = new Container(vc);
            eventContainer.parent = day_container.vc.owner;
            eventContainer.rdfStore = RdfModule.theWorld.CreateStore();
            
            AddStatement(vc, RdfVocabulary.baseUri, RdfVocabulary.sourceApplication, "datetime");

            AddStatement(vc, RdfVocabulary.baseUri, RdfVocabulary.data, date.toString());
            AddStatement(vc, RdfVocabulary.baseUri, RdfVocabulary.data, "Movie");
            AddStatement(vc, RdfVocabulary.baseUri, RdfVocabulary.data, movieTitle + "\n" + audit.toUpperCase());
            
            ParseDayEvents(day_container.vc.owner);
        }
        ReleaseStatements(statements);
    }
}

Date.prototype.getWeek = function()
{
    var janFirst = new Date(this.getFullYear(), 0, 1);
    return Math.ceil((((this - janFirst) / 86400000) + janFirst.getDay()+1)/7);
}

var date = new Date(2012, 6, 25);

var months = LOC_CAL_MONTHS;
              
var month = months[date.getMonth()];
var year = date.getFullYear();

function OnEventPress(e)
{
    if (eventsEntity)
    {
        if (this.row == 0 && this.column == 0)
        {
            // To prevent double press event when two EC_GraphicsViewCanvases a top each other. When have time, replace this with better solution.
            frame.DelayedExecute(0.2).Triggered.connect( function() { eventsEntity.placeable.visible = false; });
        }
    }
}

function OnDayPress(e)
{
    day_container.vc = this.container.visual;
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
    // construct a VisualContainer and Container.
    var visual = new VisualContainer(calendarContainer.visual);
    this.container = new Container(visual);
    this.container.parent = calendarContainer.visual.owner;
    this.container.rdfStore = RdfModule.theWorld.CreateStore();
    
    // Add visual container to calendar widget so drag & drop events will get passed.
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

    this.RdfVocabulary = RdfVocabulary;
    
    // Register QEvent listeners to given cell widget, you can find supported events in CalendarCellWidget.js file.
    if (this.widget)
    {
        if (events)
            this.widget.EventRegister = events;
    }
    MakeDraggable(visual, this.widget);
}

calendarContainer = new BaseContainer(null);
calendar_container.vc = calendarContainer.visual;
var calendar_script = new Script();
calendar_script.Invoked.connect(AddEventToCalendar);
calendarContainer.container.eventManager.RegisterScript(new Tag(RdfVocabulary.sourceApplication, "Movie"), calendar_script);
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

function AddDayCell(day, row, column, style)
{
    var cell = calendarWidget.cells[row][column];
    if (cell)
    {
        var dayLabel = new QLabel(day);
        if (style)
            dayLabel.styleSheet = style;
        cell.widget.layout().addWidget(dayLabel, 0, 0);
        var dragAddScript = new Script();
        dragAddScript.Invoked.connect(cell, DragAddEvent);
        cell.container.eventManager.RegisterScript(new Tag(RdfVocabulary.sourceApplication, "Movie"), dragAddScript);
    }
    return cell;
}

// Fill july data staticly into the calendar.
// Todo replace this with a concreate implementation.
AddDayCell("36", 1, 0); // Week
AddDayCell("1", 1, 7, Cell.SundayActiveStyle);

AddDayCell("37", 2, 0); // Week
AddDayCell("2", 2, 1);
AddDayCell("3", 2, 2);
AddDayCell("4", 2, 3);
AddDayCell("5", 2, 4);
AddDayCell("6", 2, 5);
AddDayCell("7", 2, 6);
AddDayCell("8", 2, 7, Cell.SundayActiveStyle);

AddDayCell("38", 3, 0); // Week
AddDayCell("9", 3, 1);
AddDayCell("10", 3, 2);
AddDayCell("11", 3, 3);
AddDayCell("12", 3, 4);
AddDayCell("13", 3, 5);
AddDayCell("14", 3, 6); 
AddDayCell("15", 3, 7, Cell.SundayActiveStyle);

AddDayCell("39", 4, 0); // Week
AddDayCell("16", 4, 1);
AddDayCell("17", 4, 2);
AddDayCell("18", 4, 3);
AddDayCell("19", 4, 4);
AddDayCell("20", 4, 5);
AddDayCell("21", 4, 6);
AddDayCell("22", 4, 7, Cell.SundayActiveStyle);

AddDayCell("40", 5, 0); // Week
AddDayCell("23", 5, 1);
AddDayCell("24", 5, 2);
currentDayCell = AddDayCell("25", 5, 3, Cell.DayActiveStyle);

var eventContainer = new BaseContainer(currentDayCell.container.visual);
eventContainer.container.parent = currentDayCell.container;
AddStatement(eventContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.sourceApplication, "datetime");
var date = new Date(PRESET_EVENT1_TIME[0],PRESET_EVENT1_TIME[1],PRESET_EVENT1_TIME[2],PRESET_EVENT1_TIME[3],PRESET_EVENT1_TIME[4]);
AddStatement(eventContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, date.toString());
AddStatement(eventContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, LOC_CAL_PRESET_INFO);
AddStatement(eventContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, LOC_CAL_PRESET_EVENT1);
          
eventContainer = new BaseContainer(currentDayCell.container.visual);
eventContainer.container.parent = currentDayCell.container;
AddStatement(eventContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.sourceApplication, "datetime");
date = new Date(PRESET_EVENT2_TIME[0],PRESET_EVENT2_TIME[1],PRESET_EVENT2_TIME[2],PRESET_EVENT2_TIME[3],PRESET_EVENT2_TIME[4]);
AddStatement(eventContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, date.toString());
AddStatement(eventContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, LOC_CAL_PRESET_MEETING);
AddStatement(eventContainer.visual, RdfVocabulary.baseUri, RdfVocabulary.data, LOC_CAL_PRESET_EVENT2); 

AddDayCell("26", 5, 4);
AddDayCell("27", 5, 5);
AddDayCell("28", 5, 6);
AddDayCell("29", 5, 7, Cell.SundayActiveStyle);

AddDayCell("41", 6, 0); // Week
AddDayCell("30", 6, 1);
AddDayCell("31", 6, 2);

calendarContainer.visual.size = calendarWidget.size;
calLayout.addWidget(calendarWidget, 0, 0);

me.graphicsviewcanvas.width = calendarContainer.visual.width;
me.graphicsviewcanvas.height = calendarContainer.visual.height;
me.graphicsviewcanvas.GraphicsScene().addWidget(calendarContainer.visual);
calendarContainer.visual.show();

// Todo Move code below to a separete file.

calEventsContainer = new BaseContainer(null);
event_container.vc = calEventsContainer.visual;
var eventLayout = new QHBoxLayout();
eventLayout.setSpacing(0);
eventLayout.setContentsMargins(0, 0, 0, 0);
calEventsContainer.visual.setLayout(eventLayout);

eventsEntity = scene.GetEntityByName(eventsEntityName);
var eventsWidget = asset.GetAsset("CalendarEvents.ui").Instantiate(false, 0);
calEventsContainer.visual.size = eventsWidget.size;
eventsWidget.cells = new Array();

function AddEventCellScript(row, column)
{
    var cell = eventsWidget.cells[row][column];
    if (cell)
    {
        var dragAddScript = new Script();
        cell.container.eventManager.RegisterScript(new Tag(RdfVocabulary.sourceApplication, "Movie"), dragAddScript);
        dragAddScript.Invoked.connect(cell, DragAddEventToParent);
    }
    return cell;
}

for (var i = 0; i < 4; ++i) 
{
    eventsWidget.cells[i] = [new CellObject(eventsWidget, i, 0, EventCellEvents), new CellObject(eventsWidget, i, 1, EventCellEvents)];
    AddEventCellScript(i, 0);
    AddEventCellScript(i, 1);
} 
eventsWidget.cells[0][0].widget.layout().addWidget(new QLabel("25"), 0, 0);
eventsWidget.cells[0][0].widget.styleSheet = Cell.DayEventStyle;
eventsWidget.cells[0][1].widget.layout().addWidget(new QLabel(LOC_CAL_EVENTS), 0, 0);

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
// Each day has it's own container and each day can have zero to * events containers as children.
function ParseDayEvents(container)
{
    // Check if day container has any events inside.
    var events = new Array();
    for(var i = 0; i < container.childCount; ++i)
    {
        statements = Select(container.Child(i).rdfStore, null, RdfVocabulary.data, null);
        if (statements.length == 3)
        {
            events.push(new CalendarEvent(new Date(statements[0].object.literal), statements[1].object.literal, statements[2].object.literal)); 
        }
        ReleaseStatements(statements);
    }
    
    // Remove previous events from the event widget.
    for (var i = 0; i < oldEventWidgets.length; ++i)
        oldEventWidgets[i].deleteLater();
    oldEventWidgets = new Array();
    
    // Read top three calendar events and insert them into the event widget.
    var header = null, text = null;
    for (var i = 0; i < events.length; ++i)
    {
        if (i > 2) break;
        var t = events[i].time;
        var minStr = t.getMinutes();
        if (minStr == 0)
            minStr = "00";
        var str = t.getHours() + ":" + minStr;
        header = new QLabel(str);
        var array = events[i].text.split(" ");
        var sw = array.pop();
        text = new QLabel(array.join(" ")+ "<font color=\"red\"> " + sw +"</font>");
        oldEventWidgets.push(header);
        oldEventWidgets.push(text);
        text.wordWrap = true;
        
        var tw = new QWidget();
        tw.objectName = "search";
        tw.setProperty("searchWord", sw);
        tw.setProperty("searchType", PRESET_EVENT_SEARCH);

        eventsWidget.cells[(i + 1)][0].widget.layout().addWidget(header, 0, 0);
        eventsWidget.cells[(i + 1)][1].widget.layout().addWidget(text, 0, 0);
        tw.setParent(eventsWidget.cells[(i + 1)][1].widget);
    }
}

function RefreshCalendar()
{
    if (me.graphicsviewcanvas)
        me.graphicsviewcanvas.GraphicsScene().update(me.graphicsviewcanvas.GraphicsScene().sceneRect);
    if (eventsEntity && eventsEntity.graphicsviewcanvas)
        eventsEntity.graphicsviewcanvas.GraphicsScene().update(eventsEntity.graphicsviewcanvas.GraphicsScene().sceneRect);
}

// Not best solution, but couldn't find a better solution to know when entity asset are fully loaded.
frame.DelayedExecute(5).Triggered.connect(RefreshCalendar);

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
