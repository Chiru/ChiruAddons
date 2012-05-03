#pragma once

#include "IContainer.h"
#include "3dUiModuleFwd.h"

#include <QObject>

#include <vector>

namespace CieMap
{
/// Creates instances of container class.
class ContainerFactory : public QObject
{
    Q_OBJECT

public:
    /// Creates a new instance of the container class and associates it with the specified visual container.
    /** @note sets the owner of the visual container to the new container
        @param visualContainer Visual container that should be associated with the new container */
    static IContainer *CreateContainer(IVisualContainer *visualContainer);
};

/// Generic container and a base class for other types of containers.
class Container : public IContainer
{
    friend class ContainerFactory;
    /// Default constructor should not be used.
    Container();
    /// Use ContainerFactory to create new containers
    explicit Container(IVisualContainer *vc);

    SemWeb::MemoryStore *rdfMemoryStore;
    std::vector<IContainer *> childContainers; ///< List of child containers
    IContainer *parentContainer;
    IEventManager *eventMgr;
    IVisualContainer *visualUnity;
    Layout *layout;

public:
     // IContainer overrides
    void SetRdfStore(SemWeb::MemoryStore *rdfStore) { rdfMemoryStore = rdfStore; }
    SemWeb::MemoryStore *RdfStore() const { return rdfMemoryStore; }
    IEventManager *EventManager() const { return eventMgr; }
    IVisualContainer *Visual() const { return visualUnity; }
    void SetParent(IContainer *parent);
    IContainer *Parent() const { return parentContainer; }
    int ChildCount() const { return childContainers.size(); }
    IContainer *Child(uint childIdx) const;
    bool IsInActiveRegion(IContainer *otherContainer) const;
    void DropToActive(const Tag &tag, IContainer *container);

    /// Add a child container
    /** @param c New child container that has this container as the parent */
    void AddChild(IContainer *c);

    /// Remove a child container
    /** @param c Child container */
    void RemoveChild(IContainer *c);
};

}
