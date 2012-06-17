engine.ImportExtension("qt.core");
engine.ImportExtension("qt.gui");

function Cell(size, parent)
{
    QWidget.call(this, parent);
    this.EventRegister = new Object;
    this.mouseTracking = true;
    
    this.setLayout(new QHBoxLayout());
    this.layout().setContentsMargins(0,0,0,0);
    this.layout().spacing = 0;
}
Cell.prototype = new QWidget();

Cell.prototype.mousePressEvent = function(e)
{       
    if (this.EventRegister["MousePress"])
        this.EventRegister["MousePress"].call(this, e);
    e.ignore();
}

Cell.prototype.mouseReleaseEvent = function(e)
{
    if (this.EventRegister["MouseRelease"])
        this.EventRegister["MouseRelease"].call(this, e);
    e.ignore();
}

Cell.prototype.mouseMoveEvent = function(e)
{
    if (this.EventRegister["MouseMove"])
        this.EventRegister["MouseMove"].call(this, e);
    //e.ignore();
}

Cell.DayNormalStyle = 'QLabel \
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

Cell.DayActiveStyle = 'QLabel \
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