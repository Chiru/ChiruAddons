// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "3dUiModuleFwd.h"
#include "IScriptManager.h"

#include <QString>

#include <QMap>
#include <vector>

namespace CieMap
{
/// @note Copy-paste from Tundra
int ComputeSdbmHash(const QString &str);

/// A metadata tag that is used to tie behaviour scripts to container metadata
struct Tag
{
    /// Type of the tag, f.ex. geo-location, generic metadata, datetime...
    /** Can be empty or null to match all data, but if null or empty, Data should not be null or empty. */
    QString Type() const { return type; }
    void SetType(const QString &value) { type = value; }

    /// The value of the metadata. Can be null or empty to match types, but if null or empty, Type should not be null or empty.
    QString Data() const { return data; }
    void SetData(const QString &value) { data = value; }

    /// Tests if the tag is empty, i.e. both Type and Data are empty.
    bool IsEmpty() const { return Type().isEmpty() && Data().isEmpty(); }
    static bool IsEmpty(const Tag &p) { return p.IsEmpty(); }

    /// Hash function for this type
    int GetHashCode() const { return ComputeSdbmHash(type) ^ ComputeSdbmHash(data); }

    /// Equality comparison
//    bool Equals(const Tag &rhs) const { return *this == rhs; }

    /// Returns true if type and data match, false otherwise.
    bool operator ==(const Tag &rhs) const { return Type() == rhs.Type() && Data() == rhs.Data(); }
    bool operator !=(const Tag &rhs) const { return !(*this == rhs); }
    bool operator <(const Tag &rhs) const
    {
        if (Type() < rhs.Type()) return true; else if (Type() > rhs.Type()) return false;
        if (Data() < rhs.Data()) return true; else if (Data() > rhs.Data()) return false;
        return false;
    }

    QString type;
    QString data;
};

//inline bool operator ==(const Tag &lhs, const Tag &rhs) { return lhs.operator ==(rhs); }

/// Registers scripts based on tags. Registered scripts can then be called based on tags.
class ScriptManager : public IScriptManager
{
    Q_OBJECT

public:
    ScriptManager() {}

    int RegisterScript(const Tag &tag, IScript *script);

    std::vector<int> ScriptIdsForTag(const Tag &tag) const;

    void RunScript(int id, const Tag &tag, IMemoryStore *rdfStore);

private:
    QMap<Tag, int> tags;
    std::vector<IScript*> scripts;
};

}
