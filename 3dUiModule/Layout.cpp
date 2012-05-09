// For conditions of distribution and use, see copyright notice in LICENSE

#include "Layout.h"
#include "IContainer.h"

#include <algorithm>

namespace CieMap
{

void Layout::Manage(IContainer *container)
{
    if (!container)
    {
        /// @todo print error throw new System.ArgumentException("Parameter cannot be null", "container");
        return;
    }
    managedContainers.push_back(container);
}

bool Layout::IsManaged(IContainer *container) const
{
    return std::find(managedContainers.begin(), managedContainers.end(), container) != managedContainers.end();
}

void Layout::Remove(IContainer *container)
{
    std::vector<IContainer *>::iterator it = std::find(managedContainers.begin(), managedContainers.end(), container);
    if (it == managedContainers.end())
    {
        /// @todo Print error
        return;
    }
    managedContainers.erase(it);
}

}
