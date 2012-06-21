// For conditions of distribution and use, see copyright notice in LICENSE

#include "StableHeaders.h"
#include "EC_VisualContainer.h"

EC_VisualContainer::EC_VisualContainer(Scene* scene) :
    IComponent(scene),
    container(0)
{
}

EC_VisualContainer::~EC_VisualContainer()
{
}

VisualContainer *EC_VisualContainer::GetVisualContainer() const
{
    return container;
}

void EC_VisualContainer::SetVisualContainer(VisualContainer *vc)
{
    if (container)
        disconnect(container, SIGNAL(destroyed(QObject*)));
    if (vc)
        connect(vc, SIGNAL(destroyed(QObject*)), this, SLOT(HandleVisualContainerDestroyed(QObject*)), Qt::UniqueConnection);

    container = vc;
}

void EC_VisualContainer::HandleVisualContainerDestroyed(QObject *obj)
{
    if (container == obj)
        container = 0;
}