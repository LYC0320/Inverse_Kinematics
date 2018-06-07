#include "geometry_sphere.h"

namespace geometry {

// public func.

Sphere::Sphere()
    :xfm_(new math::HomoXfm3d_t),
    radius_(double{0.0})
{
}

Sphere::Sphere(const double radius)
    :xfm_(new math::HomoXfm3d_t),
    radius_(radius)
{
}

Sphere::Sphere(const math::HomoXfm3d_t &xfm, const double radius)
    :xfm_(new math::HomoXfm3d_t(xfm)),
    radius_(radius)
{
}

Sphere::Sphere(const Sphere &other)
    :xfm_(new math::HomoXfm3d_t(other.xfm())),
    radius_(other.radius())
{
}

Sphere::~Sphere()
{
}

Sphere &Sphere::operator=(const Sphere &other)
{
    if (&other == this)
    {
        return *this;
    }

    *xfm_ = other.xfm();
    radius_ = other.radius();

    return *this;
}

math::HomoXfm3d_t Sphere::xfm() const
{
    return *xfm_;
}

double Sphere::radius() const
{
    return radius_;
}

void Sphere::set_xfm(const math::HomoXfm3d_t &xfm)
{
    *xfm_ = xfm;
}

void Sphere::set_radius(const double radius)
{
    radius_ = radius;
}

// protected func.

// private func.

} // namespace geometry {
