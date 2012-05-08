#include "VisualContainer.h"
#include "Container.h"
#include "Framework.h"
#include "3dUiModule.h"
#include "LoggingFunctions.h"

#include <QtGui>

namespace CieMap
{
    VisualContainer::VisualContainer(QWidget* parent):
        IVisualContainer(parent),
        ownerContainer(0)
    {
        setAcceptDrops(true);
    }

    VisualContainer::~VisualContainer()
    {

    }

    void VisualContainer::SetOwner(IContainer *owner)
    {
        if (ownerContainer) disconnect(this, SLOT(ParentChanged(IContainer*)));
        ownerContainer = owner;
        connect(owner, SIGNAL(ParentChanged(IContainer*)), this, SLOT(ParentChanged(IContainer*)), Qt::UniqueConnection);
    }

    IContainer* VisualContainer::Owner() const
    {
        return ownerContainer;
    }

    void VisualContainer::AttachToVisualContainer(VisualContainer* vc)
    {
        Container* cont = dynamic_cast<Container*>(vc->Owner());
        assert(cont && "Failed to dynamic cast IContainer to Container.");
        if (cont)
            cont->SetParent(Owner());
    }

    void VisualContainer::ParentChanged(IContainer* parent)
    {
        SetIgnoreDrop(true);
        setParent(parent->Visual());
    }

    void VisualContainer::dragEnterEvent(QDragEnterEvent *e)
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

    void VisualContainer::dragMoveEvent(QDragMoveEvent *e)
    {
        if (e->mimeData()->hasText())
            e->accept();
        else
             e->ignore();
    }

    void VisualContainer::dropEvent(QDropEvent *e)
    {
        VisualContainer* vc = FindVisualContainer(this);
        IContainer* parent = 0;
        while(vc)
        {
            if (!vc->IgnoreDrop())
                break;

            parent = vc->Owner()->Parent();
            if (!parent)
                break;

            vc = dynamic_cast<VisualContainer*>(vc->Owner()->Parent()->Visual());
        }

        if (vc && e->mimeData()->hasText())
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

            VisualContainer* source = FindVisualContainer(e->source());
            vc->AttachToVisualContainer(source);
            if (vc->layout())
                vc->layout()->addWidget(source);
            else
            {
                source->setParent(vc);
                source->move(position - hotSpot);
            }
            source->show();

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
    }

    VisualContainer* VisualContainer::FindVisualContainer(QWidget* widget)
    {
        QWidget* w = widget;
        while (w)
        {
            QString s = w->metaObject()->className();
            if(s == "CieMap::VisualContainer")
                return dynamic_cast<VisualContainer*>(w);
            w = w->parentWidget();
        }
        return 0;
    }

    void VisualContainer::mousePressEvent(QMouseEvent *e)
    {
        QWidget* child2 = childAt(e->pos());
        VisualContainer* vc = FindVisualContainer(child2);
        if (!vc || vc->Owner()->ChildCount() > 0) return;

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
    }

    IVisualContainer* VisualContainer::Clone()
    {
        return 0;
    }
}