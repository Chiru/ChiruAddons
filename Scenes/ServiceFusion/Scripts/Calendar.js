// !ref: CalenderWidget.ui

me.Action("Cleanup").Triggered.connect(OnScriptDestroyed);

engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

Date.prototype.getWeek = function()
{
    var janFirst = new Date(this.getFullYear(), 0, 1);
    return Math.ceil((((this - janFirst) / 86400000) + janFirst.getDay()+1)/7);
}

var date = new Date(2012, 6, 25);

var months = {0 :"Tammikuu", 1 :"Helmikuu",  2 :"Maaliskuu",
              3 :"Huhtikuu", 4 :"Toukokuu",  5 :"Kesäkuu",
              6 :"Heinäkuu", 7 :"Elokuu",    8 :"Syyskuu",
              9 :"Lokakuu",  10:"Marraskuu", 11:"Joulukuu"};
              
var month = months[date.getMonth()];
var year = date.getFullYear();

function Cell(widget, row, column, style)
{
    this.widget = findChild(widget, "cell_" + row + "_" + column);
    if (style)
        this.widget.styleSheet = style;
}
Cell.DayNormalStyle = 
'QLabel \
{ \
 color:black; \
 font: bold "FreeSans"; \
 font-size: 21px; \
 background-color:white; \
 qproperty-alignment: AlignCenter; \
} \
QLabel:hover \
{ \
 background-color:#404040; \
 color:white; \
}';

Cell.DayActiveStyle = 
'QLabel \
{ \
 color:white; \
 font: bold "FreeSans"; \
 font-size: 21px; \
 background-color:blue; \
 qproperty-alignment: AlignCenter; \
} \
QLabel:hover \
{ \
 background-color:#404040; \
 color:white; \
}';

var calendarWidget = asset.GetAsset("CalenderWidget.ui").Instantiate(false, 0);//new QCalendarWidget();
me.graphicsviewcanvas.width = calendarWidget.width;
me.graphicsviewcanvas.height = calendarWidget.height;
me.graphicsviewcanvas.GraphicsScene().addWidget(calendarWidget);
calendarWidget.show();

calendarWidget.cells = new Array();
// Get every child cell widgets from the calendar widget and store them into a double array.
// Note! cells range should be betweeen row[0-6] and column [0-7] only exeption is row[0] where column range is [0-1].
calendarWidget.cells[0] = [new Cell(), new Cell(), new Cell(), new Cell(), new Cell(), new Cell(), new Cell()];
calendarWidget.cells[0] = [new Cell(calendarWidget, 0, 0), new Cell(calendarWidget, 0, 1)];
for (var i = 1; i < 7; ++i)
{
    calendarWidget.cells[i] = [new Cell(calendarWidget, i, 0), new Cell(calendarWidget, i, 1, Cell.DayNormalStyle), new Cell(calendarWidget, i, 2, Cell.DayNormalStyle),
                               new Cell(calendarWidget, i, 3, Cell.DayNormalStyle), new Cell(calendarWidget, i, 4, Cell.DayNormalStyle), new Cell(calendarWidget, i, 5, Cell.DayNormalStyle),
                               new Cell(calendarWidget, i, 6, Cell.DayNormalStyle), new Cell(calendarWidget, i, 7, Cell.DayNormalStyle)];
}

calendarWidget.cells[0][0].widget.layout().addWidget(new QLabel(month), 0, 0);
calendarWidget.cells[0][1].widget.layout().addWidget(new QLabel(year.toString()), 0, 0);

// Fill july data staticly to calendar.
// Todo replace with concreate implementation.
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
calendarWidget.cells[5][4].widget.layout().addWidget(new QLabel("26"), 0, 0);
calendarWidget.cells[5][5].widget.layout().addWidget(new QLabel("27"), 0, 0);
calendarWidget.cells[5][6].widget.layout().addWidget(new QLabel("28"), 0, 0);
calendarWidget.cells[5][7].widget.layout().addWidget(new QLabel("29"), 0, 0);

calendarWidget.cells[6][1].widget.layout().addWidget(new QLabel("30"), 0, 0);
calendarWidget.cells[6][2].widget.layout().addWidget(new QLabel("31"), 0, 0);

function OnScriptDestroyed()
{
    if (framework.IsExiting())
        return; // Application shutting down, the widget pointers are garbage.
    if (calendarWidget)
    {
        calendarWidget.deleteLater();
        calendarWidget = null;
    }
}
