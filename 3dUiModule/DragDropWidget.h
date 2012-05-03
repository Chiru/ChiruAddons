// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include <QWidget>

/// Widget which supports drag and drop moving of labels.
/** Code adapted from Qt Draggable Text Example */
class DragDropWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DragDropWidget(QWidget *parent = 0);

protected:
    virtual void dragEnterEvent(QDragEnterEvent *event);
    virtual void dragMoveEvent(QDragMoveEvent *event);
    virtual void dropEvent(QDropEvent *event);
    virtual void mousePressEvent(QMouseEvent *event);
};
