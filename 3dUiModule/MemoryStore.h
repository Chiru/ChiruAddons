#pragma once

#include <QObject>

class IModel;
class IWorld;

namespace SemWeb
{
    /// @todo This class in C# was from SemWeb http://razor.occams.info/code/semweb/
    class MemoryStore : public QObject
    {
        Q_OBJECT
        Q_PROPERTY(IModel* store READ Model)

    public:
        MemoryStore(IWorld* world);
        virtual ~MemoryStore();

        IModel* Model() const { return model; }

    private:
        IModel* model;
    };
}
