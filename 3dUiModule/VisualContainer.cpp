// For conditions of distribution and use, see copyright notice in LICENSE

#include "VisualContainer.h"
#include "Container.h"
#include "3dUiModule.h"
#include "HttpRequestService.h"
#include "ScriptServices.h"
#include "IMemoryStore.h"
#include "IWorld.h"
#include "IStatement.h"
#include "INode.h"
#include "ScriptManager.h"

#include "LoggingFunctions.h"
#include "CoreDefines.h"

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
        SAFE_DELETE(ownerContainer)
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
        //SetIgnoreDrop(true);
        //setAcceptDrops(false);
        // todo replace this with layout functionality.
        setParent(parent->Visual());
    }

    void VisualContainer::dragEnterEvent(QDragEnterEvent *e)
    {
        if (!e->mimeData()->data("application/x-hotspot").isEmpty())
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
        if (!e->mimeData()->data("application/x-hotspot").isEmpty())
        {
            emit DragMove(e->pos(), e->mimeData()->data("application/x-hotspot"));
            e->accept();
        }
        else
             e->ignore();
    }

    void VisualContainer::dropEvent(QDropEvent *e)
    {
        VisualContainer* vc = FindVisualContainer(this);
        IContainer* parent = 0;
        while(vc)
        {
            parent = vc->Owner()->Parent();
            if (!parent)
                break;

            vc = dynamic_cast<VisualContainer*>(vc->Owner()->Parent()->Visual());
        }

        if (vc && !e->mimeData()->data("application/x-hotspot").isEmpty())
        {
            const QMimeData *mime = e->mimeData();
            QPoint position = e->pos();
            QPoint hotSpot;

            DragDrop(mime->data("application/x-hotspot"));

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
            HandleDrop(source);
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
        QWidget* child = childAt(e->pos());
        VisualContainer* vc = FindVisualContainer(child);
        if (!vc || vc->Owner()->ChildCount() > 0) return;

        QPoint hotSpot = e->pos() - child->pos();

        QMimeData *mimeData = new QMimeData;
        //if (Owner() && Owner()->RdfStore()) mimeData->setText(Owner()->RdfStore()->toString());
        mimeData->setData("application/x-hotspot", QByteArray::number(hotSpot.x()) + " " + QByteArray::number(hotSpot.y()));

        QDrag *drag = new QDrag(this);
        drag->setMimeData(mimeData);
        drag->setHotSpot(hotSpot);

        emit DragStart(mimeData->data("application/x-hotspot"));

        drag->exec(Qt::MoveAction, Qt::MoveAction);
    }

    IContainer* VisualContainer::Clone()
    {
        return 0;
    }

    void VisualContainer::HandleDrop(VisualContainer *target)
    {
        if (!target)
            return;

        IMemoryStore *store = Owner()->RdfStore();
        LogInfo(target->Owner()->RdfStore()->toString());

        IWorld *world = store->World();
        INode *source = world->CreateResource(QUrl("http://cie/source-application"));
        IStatement *statement = world->CreateStatement(0, source, 0);

        QVariantList result = store->Select(statement);

        world->FreeNode(source);
        world->FreeStatement(statement);

        for (int i = 0; i < result.count(); ++i)
        {
            IStatement *s = dynamic_cast<IStatement *>(result[i].value<QObject*>());
            Tag tag;
            tag.SetType(s->Predicate()->Uri().toString());
            tag.SetData(s->Object()->Lit());
            target->Owner()->DropToActive(tag, target->Owner());
            world->FreeStatement(s);
        }
    }
}
