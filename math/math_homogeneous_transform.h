#ifndef _MATH_HOMOGENEOUS_TRANSFORM_H_
#define _MATH_HOMOGENEOUS_TRANSFORM_H_

#include "global_def.h"
#include "math_type.h"
#undef Success ///< Avoid the conflicts with X11.h
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace math {

template<typename Scalar>
class HomogeneousTransform : public Eigen::Transform<Scalar, 3, Eigen::Isometry>
{
    typedef Eigen::Transform<Scalar, 3, Eigen::Isometry> Base_t;

public:

    typedef typename Base_t::Scalar RealScalar;
    typedef Eigen::Matrix<Scalar, 3, 3> RotMat_t;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3d_t;

    static HomogeneousTransform Identity();

    HomogeneousTransform();
    explicit HomogeneousTransform(const RotMat_t &rot_mat, const Vector3d_t &xlt);
    HomogeneousTransform(const Base_t &base);
    HomogeneousTransform(const HomogeneousTransform &other);

    virtual ~HomogeneousTransform();

    HomogeneousTransform &operator=(const HomogeneousTransform &other);


protected:

private:

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace math {

namespace math {

template<typename Scalar>
HomogeneousTransform<Scalar> HomogeneousTransform<Scalar>::Identity()
{
    return HomogeneousTransform<Scalar>();
}

template<typename Scalar>
HomogeneousTransform<Scalar>::HomogeneousTransform()
    :Base_t(Base_t::Identity())
{
}

template<typename Scalar>
HomogeneousTransform<Scalar>::HomogeneousTransform(const RotMat_t &rot_mat, const Vector3d_t &xlt)
    :Base_t(Base_t::Identity())
{
    this->linear() = rot_mat;
    this->translation() = xlt;
}

template<typename Scalar>
HomogeneousTransform<Scalar>::HomogeneousTransform(const Base_t &base)
    :Base_t(base)
{
}

template<typename Scalar>
HomogeneousTransform<Scalar>::HomogeneousTransform(const HomogeneousTransform &other)
    :Base_t(other)
{
}

template<typename Scalar>
HomogeneousTransform<Scalar>::~HomogeneousTransform()
{
}

template<typename Scalar>
HomogeneousTransform<Scalar> &HomogeneousTransform<Scalar>::operator=(const HomogeneousTransform &other)
{
    if (&other == this)
    {
        return *this;
    }

    this->linear() = other.linear();
    this->translation() = other.translation();

    return *this;
}

} // namespace math {

namespace Eigen {

template<typename charT, typename traits, typename Scalar>
std::basic_ostream<charT, traits> &operator<<(
        std::basic_ostream<charT, traits> &ostream,
        const Transform<Scalar, 3, Isometry> &isometry3d
        )
{
    ostream << "linear" << '\n'
        << isometry3d.linear() << '\n'
        << "translation: "
        << isometry3d.translation().transpose();

    return ostream;
}

}

#endif // #ifndef _MATH_HOMOGENEOUS_TRANSFORM_H_
