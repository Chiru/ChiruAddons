//Stylesheet-file for movie dialogs

var plaza2_asset = asset.GetAsset("local://plaza_2.png");
var plaza3_asset = asset.GetAsset("local://plaza_3.png");
var step_s_asset = asset.GetAsset("local://step_small.png");
var step_m_asset = asset.GetAsset("local://step_medium.png");
var step_l_asset = asset.GetAsset("local://step_large.png");
var green_seat   = asset.GetAsset("local://green_seat.png");
var yellow_seat   = asset.GetAsset("local://yellow_seat.png");
var wheel_chair   = asset.GetAsset("local://wheelchair.png");

var LargeBoldText = "QLabel { font: bold 18px 'FreeSans'; }";

var LargeText = "QLabel { font: 18px 'FreeSans'; }";

var NormalText = "QLabel { font: 16px 'FreeSans'; }";

var NormalButton = "QPushButton { font: 16px 'FreeSans'; height: 40px; }";

var RadioButton = "QRadioButton { font: 16px 'FreeSans'; }";

var PaymentArea = "QLabel { font: bold 18px 'FreeSans'; qproperty-alignment: AlignCenter; border: 2px solid black;}";

var Price = "QLabel { font: bold 18px 'FreeSans'; qproperty-alignment: AlignRight; }";

var NormalTextRightBold = "QLabel { font: bold 16px; qproperty-alignment: AlignRight; }";

var NormalTextBold = "QLabel { font: bold 16px; }";

var NormalTextRight = "QLabel { font: bold 16px; qproperty-alignment: AlignRight; }";

var LargeTextRight = "QLabel { font: 18px; qproperty-alignment: AlignRight; }";

var LargeTextRightBold = "QLabel { font: bold 18px; qproperty-alignment: AlignRight; }";

var NoSeats = "QFrame#NoSeats { padding: 0px; border: 2px solid black; border-radius: 0px; border-image: url(" + plaza3_asset.DiskSource() + "); }";

var Seats = "QFrame#Seats { padding: 0px; border: 2px solid black; border-radius: 0px; border-image: url(" + plaza2_asset.DiskSource() + "); }";

var SmallStep = "QLabel { border-image: url(" + step_s_asset.DiskSource() + "); color: white; font: 16px 'FreeSans'; qproperty-alignment: AlignCenter; }";

var MediumStep = "QLabel { border-image: url(" + step_m_asset.DiskSource() + "); color: white; font: 16px 'FreeSans'; qproperty-alignment: AlignCenter; }";

var LargeStep = "QLabel { border-image: url(" + step_l_asset.DiskSource() + "); color: white; font: 16px 'FreeSans'; qproperty-alignment: AlignCenter; }";

var EmptyStep = "QLabel { color: white; font: 16px 'FreeSans'; qproperty-alignment: AlignCenter; }";

var LargeButton = "QPushButton { font: 32px 'FreeSans'; }";

var SeatCheckBox = "QCheckBox { padding: 0px; } QCheckBox::indicator { width: 30px; height: 38px; } QCheckBox::indicator:unchecked { border-image: url(" + green_seat.DiskSource() + "); } QCheckBox::indicator:checked { border-image: url(" + yellow_seat.DiskSource() + "); }";

var HandicapSeatCheckBox = "QCheckBox { padding: 0px; } QCheckBox::indicator { width: 30px; height: 38px; } QCheckBox::indicator:unchecked { border-image: url(" + wheel_chair.DiskSource() + "); } QCheckBox::indicator:checked { border-image: url(" + wheel_chair.DiskSource() + "); }";


