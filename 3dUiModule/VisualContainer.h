// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "CieMap/IVisualContainer.h"
#include "CoreTypes.h"

/// Acts as a bridge between Tundra and the application.
class VisualContainer : public CieMap::IVisualContainer
{
    Q_OBJECT
    Q_PROPERTY(CieMap::IContainer* owner READ Owner WRITE SetOwner)
    Q_PROPERTY(entity_id_t ownerEntityId READ OwnerEntityId WRITE SetOwnerEntityId)

public:
    explicit VisualContainer(QWidget* parent = 0);
    virtual ~VisualContainer();

    /// The container that owns this visual representation
    void SetOwner(CieMap::IContainer *owner);
    CieMap::IContainer *Owner() const;

    void SetOwnerEntityId(entity_id_t id);
    entity_id_t OwnerEntityId() const;

    CieMap::IContainer* Clone();

    void AttachToVisualContainer(VisualContainer* vc);

private slots:
    void ParentChanged(CieMap::IContainer* parent);

signals:
    void DragMove(QPoint pos, QByteArray dragObject);
    void DragStart(QByteArray dragObject);
    void DragDrop(QByteArray dragObject); 

protected:
    VisualContainer* FindVisualContainer(QWidget* widget);

    virtual void dragEnterEvent(QDragEnterEvent *event);
    virtual void dragMoveEvent(QDragMoveEvent *event);
    virtual void dropEvent(QDropEvent *event);
    virtual void mousePressEvent(QMouseEvent *event);

    /// Handles drop event between two VisualContainers.
    void HandleDrop(VisualContainer *target);

    CieMap::IContainer* ownerContainer;
    entity_id_t entityId;
};
