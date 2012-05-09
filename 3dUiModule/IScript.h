// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "3dUiModuleFwd.h"

#include <QObject>

namespace CieMap
{
/// A script which can be run by EventManager based on tags
class IScript : public QObject
{
    Q_OBJECT

public:
    /// Run the script
    /** @param tag Tag that is associated with this script
        @param rdfStore RDF data that is passed to the script. */
    virtual void Run(const Tag &tag, IMemoryStore *rdfStore) = 0;
};

}
