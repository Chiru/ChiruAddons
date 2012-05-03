#pragma once

#include "3dUiModuleFwd.h"

#include <QObject>

#include <vector>

namespace CieMap
{
/// Manages child container positions in a container.
class Layout : public QObject
{
    Q_OBJECT
    Q_PROPERTY(LayoutType type READ Type WRITE SetType);
    Q_ENUMS(LayoutType)

public:
    Layout() : type(None) {}

    /// Type of layout to use for containers
    enum LayoutType
    {
        None, ///< Original positions, not moved automatically
        HorizontalList, ///< Horizontal list
        VerticalList, ///< Vertical list
        NoOverlap, ///< Original positions with best effort to make them not overlap with each other
    };

    /// Defines how containers are laid out
    LayoutType Type() const { return type; }
    void SetType(LayoutType value) { type = value; }

    /// The specified container will be managed by the layout
    /** @param container The container the layout should manage */
    Q_INVOKABLE void Manage(IContainer *container);

    /// Returns true if the specified container is managed by the layout, false otherwise.
    /** @param container The container to locate in layout */
    Q_INVOKABLE bool IsManaged(IContainer *container) const;

    /// Remove managed container from this layout. Does nothing if container is null or not managed by the layout.
    /** @param container The container that should no longer be managed by the layout */
    Q_INVOKABLE void Remove(IContainer *container);

private:
    /// List of containers managed by the layout
    std::vector<IContainer *> managedContainers;
    LayoutType type;
};

}
