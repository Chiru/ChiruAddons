// For conditions of distribution and use, see copyright notice in LICENSE

#include "Layout.h"
#include "IContainer.h"
#include "LoggingFunctions.h"

#include <algorithm>

namespace CieMap
{

void Layout::Manage(IContainer *container)
{
    if (!container)
    {
        LogError("Layout::Manage: Null container passed.");
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
        LogError("Layout::Remove: Trying to remove container from layout that does not manage the container.");
        return;
    }
    managedContainers.erase(it);
}

}
