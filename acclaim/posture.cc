#include "posture.h"
#include <algorithm>
#include "boost/numeric/conversion/cast.hpp"
#include "console_log.h"
#include "math_utils.h"
#include "math_col_vector_3d.h"

// public func.

Posture Posture::ToRadianBoneRotation(const Posture &degree_posture)
{
    math::Vector3dColl_t radian_bone_rotations = math::ToRadian(
            degree_posture.bone_rotations()
            );

    Posture radian_posture = degree_posture;
    radian_posture.set_bone_rotations(radian_bone_rotations);

    return radian_posture;
}

Posture Posture::ToDegreeBoneRotation(const Posture &radian_posture)
{
    math::Vector3dColl_t degree_bone_rotations = math::ToDegree(
            radian_posture.bone_rotations()
            );

    Posture degree_posture = radian_posture;
    degree_posture.set_bone_rotations(degree_bone_rotations);

    return degree_posture;
}

Posture::Posture()
    :size_(0),
    root_pos_(new math::Vector3d_t),
    bone_rotations_(new math::Vector3dColl_t),
    bone_translations_(new math::Vector3dColl_t)
{
}

Posture::Posture(const int32_t size)
    :size_(size),
    root_pos_(new math::Vector3d_t),
    bone_rotations_(new math::Vector3dColl_t(size, math::Vector3d_t::Zero())),
    bone_translations_(new math::Vector3dColl_t(size, math::Vector3d_t::Zero()))
{
}

Posture::Posture(const Posture &other)
    :size_(other.size()),
    root_pos_(new math::Vector3d_t(other.root_pos())),
    bone_rotations_(new math::Vector3dColl_t(other.bone_rotations())),
    bone_translations_(new math::Vector3dColl_t(other.bone_translations()))
{
}

Posture::~Posture()
{
}

Posture &Posture::operator=(const Posture &other)
{
    if (&other == this)
    {
        return *this;
    }

    size_ = other.size();
    *root_pos_ = other.root_pos();
    *bone_rotations_ = other.bone_rotations();
    *bone_translations_ = other.bone_translations();

    return *this;
}

int32_t Posture::size() const
{
    return size_;
}

math::Vector3d_t Posture::root_pos() const
{
    return *root_pos_;
}

math::Vector3dColl_t Posture::bone_rotations() const
{
    return *bone_rotations_;
}

math::Vector3d_t Posture::bone_rotation(const int32_t bone_idx) const
{
    return (*bone_rotations_)[bone_idx];
}

math::Vector3dColl_t Posture::bone_translations() const
{
    return *bone_translations_;
}

math::Vector3d_t Posture::bone_translation(const int32_t bone_idx) const
{
    return (*bone_translations_)[bone_idx];
}

void Posture::set_root_pos(const math::Vector3d_t &pos)
{
    *root_pos_ = pos;
}

void Posture::set_bone_rotations(const math::Vector3dColl_t &angle3d_coll)
{
    if (boost::numeric_cast<int>(angle3d_coll.size()) != size_)
    {
        LOGERR << "Unmatched size of angle3d_coll: " << angle3d_coll.size() << '\n'
            << "correct size : " << size_ << std::endl;
        assert(FALSE);
    }

    *bone_rotations_ = angle3d_coll;
}

void Posture::set_bone_rotation(const int32_t bone_idx, const math::Vector3d_t &angle3d)
{
    (*bone_rotations_)[bone_idx] = angle3d;
}

void Posture::set_bone_translations(const math::Vector3dColl_t &xlt_coll)
{
    *bone_translations_ = xlt_coll;
}

void Posture::set_bone_translation(const int32_t bone_idx, const math::Vector3d_t &xlt)
{
    (*bone_translations_)[bone_idx] = xlt;
}

void Posture::reset(const int32_t size)
{
    size_ = size;
    *root_pos_ = math::Vector3d_t::Zero();
    *bone_rotations_ = math::Vector3dColl_t(size_, math::Vector3d_t::Zero());
    *bone_translations_ = math::Vector3dColl_t(size_, math::Vector3d_t::Zero());
}

bool Posture::is_empty() const
{
    return 0 == size_ ? TRUE : FALSE;
}

bool Posture::is_approx(
        const Posture &other,
        const double epsilon
        ) const
{
    if (other.size() != this->size())
    {
        LOGMSG << "Unequal size" << std::endl;
        return FALSE;
    }

    math::Vector3d_t root_pos_diff = (*root_pos_) - other.root_pos();
    bool is_approx_root_pos = root_pos_diff.isZero(epsilon);

    if (!is_approx_root_pos)
    {
        LOGMSG << "Unapproximated root position" << std::endl;
        return FALSE;
    }

    bool is_approx_rotation = std::equal(
            bone_rotations_->begin(),
            bone_rotations_->end(),
            other.bone_rotations().begin(),
            [&](const math::Vector3d_t &ref_angle, const math::Vector3d_t &trg_angle)
            {return (ref_angle - trg_angle).isZero(epsilon);}
            );

    if (!is_approx_rotation)
    {
        return FALSE;
    }

    bool is_approx_xlt = std::equal(
            bone_translations_->begin(),
            bone_translations_->end(),
            other.bone_translations().begin(),
            [&](const math::Vector3d_t &ref_xlt, const math::Vector3d_t &trg_xlt)
            {return (ref_xlt - trg_xlt).isZero(epsilon);}
            );

    return is_approx_xlt;
}

// protected func.

// private func.
