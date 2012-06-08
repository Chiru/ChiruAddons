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
    CieMap::IContainer *CreateContainer(CieMap::IVisualContainer *visualContainer);
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
    ~Container();

    IMemoryStore *rdfMemoryStore;
    std::vector<IContainer *> childContainers; ///< List of child containers
    IContainer *parentContainer;
    IEventManager *eventMgr;
    IVisualContainer *visualUnity;
    CieMap::Layout *layout;

public:
     // IContainer overrides
    void SetRdfStore(IMemoryStore *rdfStore) { rdfMemoryStore = rdfStore; }
    IMemoryStore *RdfStore() const { return rdfMemoryStore; }
    CieMap::IEventManager *EventManager() const { return eventMgr; }
    CieMap::IVisualContainer *Visual() const { return visualUnity; }
    void SetParent(CieMap::IContainer *parent);
    CieMap::IContainer *Parent() const { return parentContainer; }
    int ChildCount() const { return childContainers.size(); }

    Q_INVOKABLE CieMap::IContainer *Child(uint childIdx) const;
    Q_INVOKABLE bool IsInActiveRegion(CieMap::IContainer *otherContainer) const;
    Q_INVOKABLE void DropToActive(const CieMap::Tag &tag, CieMap::IContainer *container);

    /// Add a child container
    /** @param c New child container that has this container as the parent */
    Q_INVOKABLE void AddChild(CieMap::IContainer *c);

    /// Remove a child container
    /** @param c Child container */
    Q_INVOKABLE void RemoveChild(CieMap::IContainer *c);

signals:
    void ParentChanged(IContainer* newParent);
    void ChildAdded(IContainer* child);
    void ChildAboutToBeRemoved(IContainer* child);
};

}
