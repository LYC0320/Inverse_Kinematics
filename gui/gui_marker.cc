#include "gui_marker.h"
#include "gui_color.h"

namespace gui {

Marker::Marker()
    :target_pos_(new math::Vector3d_t),
    init_pos_(new math::Vector3d_t),
    color_(new Color_t),
    is_select_(false)
{
}

Marker::~Marker()
{
}

math::Vector3d_t Marker::init_pos() const
{
    return *init_pos_;
}

math::Vector3d_t Marker::target_pos() const
{
    return *target_pos_;
}

Color_t Marker::color() const
{
    return *color_;
}

bool Marker::is_select() const
{
    return is_select_;
}

void Marker::set_color(const Color_t &color)
{
    *color_ = color;
}

void Marker::set_init_pos(const math::Vector3d_t &init_pos)
{
    *init_pos_ = init_pos;
}

void Marker::set_target_pos(const math::Vector3d_t &target_pos)
{
    *target_pos_ = target_pos;
}

void Marker::ResetPos()
{
    *target_pos_ = *init_pos_;
}

void Marker::Select()
{
    is_select_ = true;
    const Color_t new_color(
            color_->red(),
            color_->blue(),
            color_->green(),
            color_->alpha()
            );
    this->set_color(new_color);
}

void Marker::Release()
{
    this->is_select_ = false;
    const Color_t new_color(
            color_->red(),
            color_->blue(),
            color_->green(),
            color_->alpha()
            );
    this->set_color(new_color);
}

void Marker::Move(const math::Vector3d_t &shift_vector)
{
    *target_pos_ += shift_vector;
}

} // namespace gui {
