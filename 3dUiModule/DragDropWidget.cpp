// For conditions of distribution and use, see copyright notice in LICENSE

#include "DragDropWidget.h"

#include <QtGui>

#include "LoggingFunctions.h"

DragDropWidget::DragDropWidget(QWidget *parent) : QWidget(parent)
{
    setAcceptDrops(true);
//    setMouseTracking(true);
}

void DragDropWidget::dragEnterEvent(QDragEnterEvent *e)
{
    if (e->mimeData()->hasText())
    {
        if (children().contains(e->source()))
        {
            e->setDropAction(Qt::MoveAction);
            e->accept();
        }
        else
        {
            e->acceptProposedAction();
        }
    }
    else
    {
        e->ignore();
    }
}

void DragDropWidget::dragMoveEvent(QDragMoveEvent *e)
{
    if (e->mimeData()->hasText())
        e->accept();
    else
         e->ignore();
}

void DragDropWidget::dropEvent(QDropEvent *e)
{
    if (e->mimeData()->hasText())
    {
        const QMimeData *mime = e->mimeData();
        QPoint position = e->pos();
        QPoint hotSpot;

        QList<QByteArray> hotSpotPos = mime->data("application/x-hotspot").split(' ');
        if (hotSpotPos.size() == 2)
        {
            hotSpot.setX(hotSpotPos.first().toInt());
            hotSpot.setY(hotSpotPos.last().toInt());
        }

        QLabel *newLabel = new QLabel(mime->text(), this);
        newLabel->setAttribute(Qt::WA_DeleteOnClose);
        if (layout())
            layout()->addWidget(newLabel);
        else
            newLabel->move(position - hotSpot);

        newLabel->show();

        if (e->source() == this)
        {
            e->setDropAction(Qt::MoveAction);
            e->accept();
        }
        else
        {
            e->setDropAction(Qt::MoveAction);
            e->acceptProposedAction();
        }
    }
    else
    {
        e->ignore();
    }

    foreach(QObject *child, children())
    {
        if (child->inherits("QWidget"))
        {
            QWidget *widget = static_cast<QWidget *>(child);
            if (!widget->isVisible())
                widget->deleteLater();
        }
    }
}

void DragDropWidget::mousePressEvent(QMouseEvent *e)
{
    QLabel *child = static_cast<QLabel*>(childAt(e->pos()));
    if (!child)
        return;

    QPoint hotSpot = e->pos() - child->pos();

    QMimeData *mimeData = new QMimeData;
    mimeData->setText(child->text());
    mimeData->setData("application/x-hotspot", QByteArray::number(hotSpot.x()) + " " + QByteArray::number(hotSpot.y()));

    QPixmap pixmap(child->size());
    child->render(&pixmap);

    QDrag *drag = new QDrag(this);
    drag->setMimeData(mimeData);
    drag->setPixmap(pixmap);
    drag->setHotSpot(hotSpot);

    Qt::DropAction dropAction = drag->exec(Qt::MoveAction, Qt::MoveAction);
    if (dropAction == Qt::MoveAction)
        child->close();
}
