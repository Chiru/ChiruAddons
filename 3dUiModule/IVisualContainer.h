// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "3dUiModuleFwd.h"
#include "Position3.h"
#include <QWidget>

namespace CieMap
{
/// Visual representation of a container.
class IVisualContainer : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(bool noDrop READ IgnoreDrop WRITE SetIgnoreDrop)

public:
    explicit IVisualContainer(QWidget* parent = 0):
        QWidget(parent),
        ignoreDrop(false)
    {
    }

    /// Position of the visual container in the 3d virtual world
    virtual void SetPosition(const Position3 &pos) { position = pos; }
    Position3 Position() const { return position; }

    /// The container that owns this visual representation
    virtual void SetOwner(IContainer *owner) = 0;
    virtual IContainer *Owner() const = 0;

    void SetIgnoreDrop(bool value) { ignoreDrop = value; }
    bool IgnoreDrop() const {return ignoreDrop;}

    /// Tests if the specified visual container is inside the active region of the visual container.
    /** @param other Position to test
        @return true if the other visual container is in the active region of the visual container, false otherwise. */
    //virtual bool IsInsideActiveRegion(IVisualContainer *other) = 0;

private:
    QVector3D position;
    bool ignoreDrop;
};

}
