#pragma once

#include "3dUiModuleFwd.h"

#include <QObject>

namespace CieMap
{

/// Container of containers.
/** A container is an element within the scene the user can interact with.
    Containers form a hierarchy which is managed by itself. The container that has no parent is the root container.
    Child containers can be accessed using an index. Order of child containers is arbitrary. */
class IContainer : public QObject
{
    Q_OBJECT
    Q_PROPERTY(IContainer* parent READ Parent WRITE SetParent);
    Q_PROPERTY(SemWeb::MemoryStore* rdfStore READ RdfStore WRITE SetRdfStore);
    Q_PROPERTY(IEventManager* eventManager READ EventManager);
    Q_PROPERTY(IVisualContainer* visual READ Visual);
    Q_PROPERTY(int childCount READ ChildCount);

public:
    IContainer() {}
    virtual ~IContainer() {}

    /// The parent of this container.
    /** You can add a container into the hierarchy by setting the parent to a container that is
        already in the hierarchy.
        Set to null to detach container from the hierarchy
        The root container has no parent. Every other container in the hierarchy must have a parent. */
    virtual IContainer *Parent() const = 0;
    virtual void SetParent(IContainer *parent) = 0;

    /// The RDF data that is associated with the container
    virtual SemWeb::MemoryStore *RdfStore() const = 0;
    virtual void SetRdfStore(SemWeb::MemoryStore *store) = 0;

    /// Event manager for this container. Shared by all containers.
    virtual IEventManager *EventManager() const = 0;

    /// Visual container associated with the container.
    virtual IVisualContainer *Visual() const = 0;

    /// The number of child containers the container has.
    virtual int ChildCount() const = 0;

    /// Returns a child container based on the index.
    /** @param childIdx Child index
        @return child container */
    Q_INVOKABLE virtual IContainer *Child(uint childIdx) const = 0;

    /// Returns true if the specified container is inside the active region of the container
    /** Throws ArgumentException if otherContainer is null, and NullReferenceException if
        the visual container is null either for this or otherContainer.
        @param otherContainer
        @return True if the specified container is inside the active region of the container, false otherwise */
    Q_INVOKABLE virtual bool IsInActiveRegion(IContainer *otherContainer) const = 0;

    /// Drop a tag to the active region of the container.
    /** This is a shortcut method and equal to calling IContainer::EventManager::CallScript().
        @param tag associated tag
        @param container Container the event originates from. This is passed to the script that is called. */
    Q_INVOKABLE virtual void DropToActive(const Tag &tag, IContainer *container) = 0;

private:
    IContainer *parent;
};

}
