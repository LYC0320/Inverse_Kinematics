#include "gui_polar_rotation.h"

namespace gui {

// public func.

PolarRotation::PolarRotation()
    :azimuth_(0.0),
    elevation_(0.0),
    twist_(0.0)
{
}

PolarRotation::PolarRotation(
        const double azimuth,
        const double elevation,
        const double twist
        )
    :azimuth_(azimuth),
    elevation_(elevation),
    twist_(twist)
{
}

PolarRotation::PolarRotation(const PolarRotation &other)
    :azimuth_(other.azimuth()),
    elevation_(other.elevation()),
    twist_(other.twist())
{
}

PolarRotation::~PolarRotation()
{
}

PolarRotation &PolarRotation::operator=(const PolarRotation &other)
{
    if (&other == this)
    {
        return *this;
    }

    azimuth_ = other.azimuth();
    elevation_ = other.elevation();
    twist_ = other.twist();

    return *this;
}

double PolarRotation::azimuth() const
{
    return azimuth_;
}

double PolarRotation::elevation() const
{
    return elevation_;
}

double PolarRotation::twist() const
{
    return twist_;
}

void PolarRotation::set(
        const double azimuth,
        const double elevation,
        const double twist
        )
{
    azimuth_ = azimuth;
    elevation_ = elevation;
    twist_ = twist;
}

void PolarRotation::set_azimuth(const double azimuth)
{
    azimuth_ = azimuth;
}

void PolarRotation::set_elevation(const double elevation)
{
    elevation_ = elevation;
}

void PolarRotation::set_twist(const double twist)
{
    twist_ = twist;
}

// protected func.

// private func.

} // namespace gui {
