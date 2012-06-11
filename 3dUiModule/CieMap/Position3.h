// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include <QVector3D>

#include <QObject>

namespace CieMap
{
/// 3D position in world space.
typedef QVector3D Position3;
/*
class Position3 : public QObject
{
    Q_OBJECT

public:
    /// Defaults to zero position (0,0,0)
    Position3() : x(0.0f), y(0.0f), z(0.0f) {}
    Position3(float xValue, float yValue, float zValue) : x(xValue), y(yValue), z(zValue) {}

    /// X position
    void SetX(float value) { x = value; }
    float X() const { return x; }

    /// Y position
    void SetY(float value) { y = value; }
    float Y() const { return y; }

    /// Z position
    void SetZ(float value) { z = value; }
    float Z() const { return z; }

    // TODO: ToString,
    // TODO FromString

    float x, y, z;
};
*/
}
