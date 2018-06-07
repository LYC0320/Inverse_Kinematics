#include "gui_camera.h"
#include "math_col_vector_3d.h"
#include "gui_polar_rotation.h"

namespace gui {

// public func.

Camera::Camera()
    :zoom_(1.0),
    polar_rotation_(new PolarRotation),
    translation_(new math::Vector3d_t),
    look_at_pos_(new math::Vector3d_t)
{
}

Camera::Camera(const Camera &other)
    :zoom_(other.zoom()),
    polar_rotation_(new PolarRotation(other.polar_rotation())),
    translation_(new math::Vector3d_t(other.translation())),
    look_at_pos_(new math::Vector3d_t(other.look_at_pos()))
{
}

Camera::~Camera()
{
}

Camera &Camera::operator=(const Camera &other)
{
    if (&other == this)
    {
        return *this;
    }

    zoom_ = other.zoom();
    *polar_rotation_ = other.polar_rotation();
    *translation_ = other.translation();
    *look_at_pos_ = other.look_at_pos();

    return *this;
}

double Camera::zoom() const
{
    return zoom_;
}

double Camera::azimuth() const
{
    return polar_rotation_->azimuth();
}

double Camera::elevation() const
{
    return polar_rotation_->elevation();
}

double Camera::twist() const
{
    return polar_rotation_->twist();
}

PolarRotation Camera::polar_rotation() const
{
    return *polar_rotation_;
}

math::Vector3d_t Camera::translation() const
{
    return *translation_;
}

math::Vector3d_t Camera::look_at_pos() const
{
    return *look_at_pos_;
}

void Camera::set_zoom(const double zoom)
{
    zoom_ = zoom;
}

//void Camera::setRotation(
//        const double azimuth,
//        const double elevation,
//        const double twist
//        )
//{
//    polar_rotation_->set(
//            azimuth,
//            elevation,
//            twist
//            );
//}

void Camera::ShiftRotation(
        const double azimuth,
        const double elevation,
        const double twist
        )
{
    polar_rotation_->set(
            polar_rotation_->azimuth() + azimuth,
            polar_rotation_->elevation() + elevation,
            polar_rotation_->twist() + twist
            );
}

void Camera::set_polar_rotation(const PolarRotation &polar_rotation)
{
    *polar_rotation_ = polar_rotation;
}

void Camera::set_translation(const math::Vector3d_t &translation)
{
    *translation_ = translation;
}

void Camera::set_look_at_pos(const math::Vector3d_t &look_at_pos)
{
    *look_at_pos_ = look_at_pos;
}

void Camera::ShiftRotation(const PolarRotation &polar_rotation)
{
    this->ShiftRotation(
            polar_rotation.azimuth(),
            polar_rotation.elevation(),
            polar_rotation.twist()
            );
}

void Camera::ShiftTranslation(const math::Vector3d_t &shift_translation)
{
    *translation_ += shift_translation;
}

void Camera::ShiftLookAt(const math::Vector3d_t &shift_pos)
{
    *look_at_pos_ += shift_pos;
}

// protected func.

// private func.

} // namespace gui {
