// For conditions of distribution and use, see copyright notice in LICENSE

#include "Container.h"
#include "EventManager.h"
#include "Layout.h"
#include "IVisualContainer.h"
#include "LoggingFunctions.h"
#include "IMemoryStore.h"
#include "IWorld.h"

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

Container::~Container()
{
    if (eventMgr) delete eventMgr;
    if (rdfMemoryStore) rdfMemoryStore->World()->FreeStore(rdfMemoryStore);
}

CieMap::IContainer *ContainerFactory::CreateContainer(CieMap::IVisualContainer *visualContainer)
{
    IContainer *container =  new Container(visualContainer);
    visualContainer->SetOwner(container);
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
        LogError("index out of bound");
        return 0;
    }
    return childContainers[childIdx];
}

bool Container::IsInActiveRegion(IContainer *otherContainer) const
{
    if (!Visual())
    {
        LogError("Visual container cannot be null");
        /// @todo Print error throw new NullReferenceException("Visual container cannot be null");
        return false;
    }
    if (!otherContainer)
    {
        LogError("Parameter cannot be null");
        /// @todo Print error throw new System.ArgumentException("Parameter cannot be null", "otherContainer");
        return false;
    }
    if (!otherContainer->Visual())
    {
        LogError("otherContainer.Visual cannot be null");
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
        LogError("Parameter cannot be null");
        /// @todo Print error throw new System.ArgumentNullException("Parameter cannot be null", "c");
        return;
    }
    
    std::vector<IContainer *>::iterator it = std::find(childContainers.begin(), childContainers.end(), c);
    if (it != childContainers.end())
    {
        LogError("Container already have parameter as child");
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
        LogError("Couldn't find a child in container");
        /// @todo Print error
        return;
    }
    emit ChildAboutToBeRemoved(c);
    childContainers.erase(it);
    layout->Remove(c);
}

}
