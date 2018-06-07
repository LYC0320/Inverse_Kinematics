#include "geometry_ellipsoid.h"

namespace geometry {

// public func.

Ellipsoid::Ellipsoid()
    :xfm_(new math::HomoXfm3d_t(math::HomoXfm3d_t::Identity())),
    size_(new math::Vector3d_t)
{
}

Ellipsoid::Ellipsoid(const math::Vector3d_t &size)
    :xfm_(new math::HomoXfm3d_t(math::HomoXfm3d_t::Identity())),
    size_(new math::Vector3d_t(size))
{
}

Ellipsoid::Ellipsoid(const math::HomoXfm3d_t &xfm, const math::Vector3d_t &size)
    :xfm_(new math::HomoXfm3d_t(xfm)),
    size_(new math::Vector3d_t(size))
{
}

Ellipsoid::Ellipsoid(const Ellipsoid &other)
    :xfm_(new math::HomoXfm3d_t(other.xfm())),
    size_(new math::Vector3d_t(other.size()))
{
}

Ellipsoid::~Ellipsoid()
{
}

Ellipsoid &Ellipsoid::operator=(const Ellipsoid &other)
{
    if (&other == this)
    {
        return *this;
    }

    *xfm_ = other.xfm();
    *size_ = other.size();

    return *this;
}

math::HomoXfm3d_t Ellipsoid::xfm() const
{
    return *xfm_;
}

math::Vector3d_t Ellipsoid::size() const
{
    return *size_;
}

void Ellipsoid::set_xfm(const math::HomoXfm3d_t &xfm)
{
    *xfm_ = xfm;
}

void Ellipsoid::set_size(const math::Vector3d_t &size)
{
    *size_ = size;
}

// protected func.

// private func.

} // namespace geometry {
