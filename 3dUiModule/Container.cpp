// For conditions of distribution and use, see copyright notice in LICENSE

#include "Container.h"
#include "EventManager.h"
#include "Layout.h"
#include "IVisualContainer.h"

#include <cassert>
#include <algorithm>

namespace CieMap
{
Container::Container(IVisualContainer *vc) :
    rdfMemoryStore(0),
    visualUnity(vc),
    layout(new Layout()),
    parentContainer(0)
{
    eventMgr = new CieMap::EventManager();
}

CieMap::IContainer *ContainerFactory::CreateContainer(CieMap::IVisualContainer *visualContainer, CieMap::IVisualContainer* parent)
{
    IContainer *container =  new Container(visualContainer);
    visualContainer->SetOwner(container);
    if (parent)
        container->SetParent(parent->Owner());
    return container;
}

void Container::SetParent(IContainer *parent)
{
    if (parent != parentContainer)
    {
        if (parentContainer)
        {
            assert(static_cast<Container *>(parentContainer));
            static_cast<Container *>(parentContainer)->RemoveChild(this);
        }
        parentContainer = parent;
        if (parentContainer)
        {
            assert(static_cast<Container *>(parentContainer));
            static_cast<Container *>(parentContainer)->AddChild(this);
            emit ParentChanged(parent);
        }
    }
}

IContainer *Container::Child(uint childIdx) const
{
    if (childIdx < 0 || childIdx >= childContainers.size())
    {
        /// @todo Print error
        return 0;
    }
    return childContainers[childIdx];
}

bool Container::IsInActiveRegion(IContainer *otherContainer) const
{
    if (!Visual())
    {
        /// @todo Print error throw new NullReferenceException("Visual container cannot be null");
        return false;
    }
    if (!otherContainer)
    {
        /// @todo Print error throw new System.ArgumentException("Parameter cannot be null", "otherContainer");
        return false;
    }
    if (!otherContainer->Visual())
    {
        /// @todo Print error throw new NullReferenceException("otherContainer.Visual cannot be null");
        return false;
    }
    //return Visual()->IsInsideActiveRegion(otherContainer->Visual());
    //\todo Add implementation --Joosua.
    return true;
}

void Container::DropToActive(const Tag &tag, IContainer *container)
{
    EventManager()->CallScript(tag, container->RdfStore());
}

void Container::AddChild(IContainer *c)
{
    if (!c)
    {
        /// @todo Print error throw new System.ArgumentNullException("Parameter cannot be null", "c");
        return;
    }
    
    std::vector<IContainer *>::iterator it = std::find(childContainers.begin(), childContainers.end(), c);
    if (it != childContainers.end())
    {
        /// @todo print error
        return;
    }

    childContainers.push_back(c);
    layout->Manage(c);
    emit ChildAdded(c);
}

void Container::RemoveChild(IContainer *c)
{
    std::vector<IContainer *>::iterator it = std::find(childContainers.begin(), childContainers.end(), c);
    if (it == childContainers.end())
    {
        /// @todo Print error
        return;
    }
    emit ChildAboutToBeRemoved(c);
    childContainers.erase(it);
    layout->Remove(c);
}

}
