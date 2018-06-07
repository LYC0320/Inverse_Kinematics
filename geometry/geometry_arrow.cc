#include "geometry_arrow.h"

namespace geometry {

// public func.

Arrow::Arrow()
    :start_pos_(new math::Vector3d_t),
    end_pos_(new math::Vector3d_t),
    cylinder_height_(0.0),
    cone_height_(0.0),
    cylinder_radius_(0.0),
    cone_radius_(0.0)
{
}

Arrow::Arrow(const math::Vector3d_t &start_pos, const math::Vector3d_t &end_pos)
    :start_pos_(new math::Vector3d_t(start_pos)),
    end_pos_(new math::Vector3d_t(end_pos)),
    cylinder_height_(0.0),
    cone_height_(0.0),
    cylinder_radius_(0.0),
    cone_radius_(0.0)
{
}

Arrow::Arrow(const Arrow &other)
    :start_pos_(new math::Vector3d_t(other.start_pos())),
    end_pos_(new math::Vector3d_t(other.end_pos())),
    cylinder_height_(other.cylinder_height()),
    cone_height_(other.cone_height()),
    cylinder_radius_(other.cylinder_radius()),
    cone_radius_(other.cone_radius())
{
}

Arrow::~Arrow()
{
}

Arrow &Arrow::operator=(const Arrow &other)
{
    if (&other == this)
    {
        return *this;
    }

    *start_pos_ = other.start_pos();
    *end_pos_ = other.end_pos();
    cylinder_height_ = other.cylinder_height();
    cone_height_ = other.cone_height();
    cylinder_radius_ = other.cylinder_radius();
    cone_radius_ = other.cone_radius();

    return *this;
}

math::Vector3d_t Arrow::start_pos() const
{
    return *start_pos_;
}

math::Vector3d_t Arrow::end_pos() const
{
    return *end_pos_;
}

double Arrow::cylinder_height() const
{
    return cylinder_height_;
}

double Arrow::cone_height() const
{
    return cone_height_;
}

double Arrow::cylinder_radius() const
{
    return cylinder_radius_;
}

double Arrow::cone_radius() const
{
    return cone_radius_;
}

void Arrow::set_start_pos(const math::Vector3d_t &start_pos)
{
    *start_pos_ = start_pos;
}

void Arrow::set_end_pos(const math::Vector3d_t &end_pos)
{
    *end_pos_ = end_pos;
}

void Arrow::set_cylinder_height(const double cylinder_height)
{
    cylinder_height_ = cylinder_height;
}

void Arrow::set_cone_height(const double cone_height)
{
    cone_height_ = cone_height;
}

void Arrow::set_cylinder_radius(const double cylinder_radius)
{
    cylinder_radius_ = cylinder_radius;
}

void Arrow::set_cone_radius(const double cone_radius)
{
    cone_radius_ = cone_radius;
}

// protected func.

// private func.

} // namespace geom {
