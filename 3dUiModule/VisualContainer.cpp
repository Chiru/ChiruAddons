// For conditions of distribution and use, see copyright notice in LICENSE

#include "VisualContainer.h"
#include "3dUiModule.h"

#include "CieMap/Container.h"
#include "CieMap/HttpRequestService.h"
#include "CieMap/ScriptServices.h"
#include "CieMap/ScriptManager.h"

#include "IMemoryStore.h"
#include "IWorld.h"
#include "IStatement.h"
#include "INode.h"

#include "LoggingFunctions.h"
#include "CoreDefines.h"

#include <QtGui>

Q_DECLARE_METATYPE(IStatement *)

VisualContainer::VisualContainer(QWidget* parent):
    IVisualContainer(parent),
    ownerContainer(0),
    cloneOnDrag(false)
{
    setAcceptDrops(true);
    setMouseTracking(true);
}

VisualContainer::~VisualContainer()
{
    SAFE_DELETE(ownerContainer)
}

void VisualContainer::SetOwner(CieMap::IContainer *owner)
{
    if (ownerContainer)
        disconnect(this, SLOT(ParentChanged(CieMap::IContainer *)));
    ownerContainer = owner;
    connect(owner, SIGNAL(ParentChanged(CieMap::IContainer *)), this, SLOT(ParentChanged(CieMap::IContainer *)), Qt::UniqueConnection);
}

CieMap::IContainer * VisualContainer::Owner() const
{
    return ownerContainer;
}

bool VisualContainer::UseCloneOnDrag() const
{
    return cloneOnDrag;
}

void VisualContainer::SetUseCloneOnDrag(bool use)
{
    cloneOnDrag = use;
}

void VisualContainer::AttachToVisualContainer(VisualContainer* vc)
{
    CieMap::Container * cont = dynamic_cast<CieMap::Container *>(vc->Owner());
    assert(cont && "Failed to dynamic cast IContainer to Container.");
    if (cont)
        cont->SetParent(Owner());
}

void VisualContainer::HandleMeshDrop(VisualContainer *target)
{
    HandleDrop(target);
}

void VisualContainer::StartDrag(QPoint pos)
{
    QMimeData *mimeData = new QMimeData;
    //if (Owner() && Owner()->RdfStore()) mimeData->setText(Owner()->RdfStore()->toString());
    mimeData->setData("application/x-hotspot", QByteArray::number(pos.x()) + " " + QByteArray::number(pos.y()));

    QDrag *drag = new QDrag(this);
    drag->setMimeData(mimeData);
    drag->setHotSpot(pos);

    emit DragStartEvent(0);

    drag->exec(Qt::MoveAction, Qt::MoveAction);
}

void VisualContainer::ParentChanged(CieMap::IContainer * parent)
{
    //SetIgnoreDrop(true);
    //setAcceptDrops(false);
    /// @todo replace this with layout functionality.
    //setParent(parent->Visual());
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
        emit DragMoveEvent(e);
        e->accept();
    }
    else
         e->ignore();
}

void VisualContainer::dropEvent(QDropEvent *e)
{
    VisualContainer* vc = FindVisualContainer(this);

    if (vc && !e->mimeData()->data("application/x-hotspot").isEmpty())
    {
        const QMimeData *mime = e->mimeData();
        QPoint position = e->pos();
        QPoint hotSpot;

        emit DropEvent(e);

        QList<QByteArray> hotSpotPos = mime->data("application/x-hotspot").split(' ');
        if (hotSpotPos.size() == 2)
        {
            hotSpot.setX(hotSpotPos.first().toInt());
            hotSpot.setY(hotSpotPos.last().toInt());
        }

        VisualContainer* source = FindVisualContainer(e->source());
        HandleDrop(source);
        emit DropEvent(e);
        
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
    QString className = metaObject()->className();
    while (w)
    {
        QString s = w->metaObject()->className();
        if(s == className)
            return dynamic_cast<VisualContainer*>(w);
        w = w->parentWidget();
    }
    return 0;
}

void VisualContainer::mousePressEvent(QMouseEvent *e)
{
    QWidget* child = childAt(e->pos());
    VisualContainer* vc = FindVisualContainer(child);
    if (!vc || vc->Owner()->ChildCount() > 0)
        return;

    QPoint hotSpot = e->pos() - child->pos();
    StartDrag(hotSpot);
}

CieMap::IContainer * VisualContainer::Clone()
{
    VisualContainer *visualClone = new VisualContainer();
    CieMap::IContainer *clone = CieMap::ContainerFactory::CreateContainer(visualClone);
    clone->SetRdfStore(Owner()->RdfStore()->Clone());
    /// @todo add children cloning aswell.
    return clone;
}

void VisualContainer::HandleDrop(VisualContainer *target)
{
    if (!target && target == this)
        return;

    if (cloneOnDrag)
    {
        CieMap::IContainer *clone = target->Clone();
        assert(clone && "HandleDrop(VisualContainer*): Failed to create a clone of VisualContainer.");
        if (clone)
            clone->SetParent(Owner());
        else
            LogError("HandleDrop(VisualContainer*): Failed to create a clone of VisualContainer.");
    }

    IMemoryStore *store = target->Owner()->RdfStore();

    IWorld *world = store->World();

    IStatement *statement = world->CreateStatement(0, 0, 0);
    QVariantList result = store->Select(statement);
    world->FreeStatement(statement);

    for (int i = 0; i < result.count(); ++i)
    {
        IStatement *s = result[i].value<IStatement *>();
        if (s)
        {
            CieMap::Tag tag;
            tag.SetType(s->Predicate()->Uri().toString());
            tag.SetData(s->Object()->Lit());
            Owner()->DropToActive(tag, target->Owner());
            world->FreeStatement(s); 
        }
    }
}
