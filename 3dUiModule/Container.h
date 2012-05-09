// For conditions of distribution and use, see copyright notice in LICENSE

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

public slots:
    /// Creates a new instance of the container class and associates it with the specified visual container.
    /** @note sets the owner of the visual container to the new container
        @param visualContainer Visual container that should be associated with the new container */
    CieMap::IContainer *CreateContainer(CieMap::IVisualContainer *visualContainer, CieMap::IVisualContainer* parent = 0);
};

/// Generic container and a base class for other types of containers.
class Container : public IContainer
{
    Q_OBJECT
    friend class ContainerFactory;
    /// Default constructor should not be used.
    Container();
    /// Use ContainerFactory to create new containers
    explicit Container(IVisualContainer *vc);

    RdfMemoryStore *rdfMemoryStore;
    std::vector<IContainer *> childContainers; ///< List of child containers
    IContainer *parentContainer;
    IEventManager *eventMgr;
    IVisualContainer *visualUnity;
    Layout *layout;

public:
     // IContainer overrides
    void SetRdfStore(RdfMemoryStore *rdfStore) { rdfMemoryStore = rdfStore; }
    RdfMemoryStore *RdfStore() const { return rdfMemoryStore; }
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

signals:
    void ParentChanged(IContainer* newParent);
    void ChildAdded(IContainer* child);
    void ChildAboutToBeRemoved(IContainer* child);
};

}
