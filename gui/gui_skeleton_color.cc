#include "gui_skeleton_color.h"
#include "boost/numeric/conversion/cast.hpp"
#include "gui_color.h"

namespace gui {

// public func.

SkeletonColor::SkeletonColor()
    :color_coll_(new ColorColl_t)
{
}

SkeletonColor::SkeletonColor(const SkeletonColor &other)
    :color_coll_(new ColorColl_t(other.color_coll()))
{
}

SkeletonColor::~SkeletonColor()
{
}

SkeletonColor &SkeletonColor::operator=(const SkeletonColor &other)
{
    if (&other == this)
    {
        return *this;
    }

    *color_coll_ = other.color_coll();

    return *this;
}

Color_t SkeletonColor::operator()(const int32_t skeleton_idx) const
{
    const int32_t idx
        = skeleton_idx
        % boost::numeric_cast<int32_t>(color_coll_->size());

    return color_coll_->at(idx);
}

ColorColl_t SkeletonColor::color_coll() const
{
    return *color_coll_;
}

void SkeletonColor::Configure(const ColorColl_t &color_coll)
{
    *color_coll_ = color_coll;
}

// protected func.

// private func.

} // namespace gui {
