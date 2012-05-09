// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "IVisualContainer.h"
#include "CoreTypes.h"

class Framework;

namespace CieMap
{
    class IContainer;

    class VisualContainer : public IVisualContainer
    {
        Q_OBJECT
        Q_PROPERTY(CieMap::IContainer* owner READ Owner WRITE SetOwner)

    public:
        explicit VisualContainer(QWidget* parent = 0);
        virtual ~VisualContainer();

        /// The container that owns this visual representation
        void SetOwner(IContainer *owner);
        IContainer *Owner() const;

        IVisualContainer* Clone();

        void AttachToVisualContainer(VisualContainer* vc);

    private slots:
        void ParentChanged(IContainer* parent);

    protected:
        VisualContainer* FindVisualContainer(QWidget* widget);

        virtual void dragEnterEvent(QDragEnterEvent *event);
        virtual void dragMoveEvent(QDragMoveEvent *event);
        virtual void dropEvent(QDropEvent *event);
        virtual void mousePressEvent(QMouseEvent *event);

       IContainer* ownerContainer;
    };
}
