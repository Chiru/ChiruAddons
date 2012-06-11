// For conditions of distribution and use, see copyright notice in LICENSE

#include "Container.h"
#include "EventManager.h"
#include "Layout.h"
#include "IVisualContainer.h"
#include "LoggingFunctions.h"
#include "CoreDefines.h"
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
    SAFE_DELETE(eventMgr)
    if (rdfMemoryStore && rdfMemoryStore->World())
        rdfMemoryStore->World()->FreeStore(rdfMemoryStore);
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
        LogError("Container::Child: Out-of-range index given: " + QString::number(childIdx));
        return 0;
    }
    return childContainers[childIdx];
}

bool Container::IsInActiveRegion(IContainer *otherContainer) const
{
    if (!Visual())
    {
        LogError("Container::IsInActiveRegion: This container doesn't have a visual container.");
        return false;
    }
    if (!otherContainer)
    {
        LogError("Container::IsInActiveRegion: Null container passed.");
        return false;
    }
    if (!otherContainer->Visual())
    {
        LogError("Container::IsInActiveRegion: Passed container doesn't have a visual container.");
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
        LogError("Container::AddChild: Null container passed.");
        return;
    }
    
    std::vector<IContainer *>::iterator it = std::find(childContainers.begin(), childContainers.end(), c);
    if (it != childContainers.end())
    {
        LogError("Container::AddChild: Trying to add container as a child, but it's already a child of this container.");
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
        LogError("Container::RemoveChild: Trying to remove container that is not child of this container.");
        return;
    }
    emit ChildAboutToBeRemoved(c);
    childContainers.erase(it);
    layout->Remove(c);
}

}
