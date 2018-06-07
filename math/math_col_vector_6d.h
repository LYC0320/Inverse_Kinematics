#ifndef _MATH_COL_VECTOR_6D_H_
#define _MATH_COL_VECTOR_6D_H_

#include <iostream>
#undef Success ///< Avoid the conflicts with X11.h
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "math_col_vector_3d.h"

namespace math {

/**
 * \brief The 6D column vector.
 * The angular part is the top 3 elements, and the linear part is the bottom 3 elements.
 */
template<typename Scalar>
class ColVector6d : public Eigen::Matrix<Scalar, 6, 1>
{
    typedef Eigen::Matrix<Scalar, 6, 1> Base_t;

public:

    typedef typename Base_t::RealScalar RealScalar;
    typedef ColVector3d<Scalar> Vector3d_t;

    ColVector6d();
    explicit ColVector6d(const Vector3d_t &rot_vector, const Vector3d_t &xlt_vector);
    template<typename OtherDerived>
    ColVector6d(const Eigen::MatrixBase<OtherDerived> &other);
    virtual ~ColVector6d();

    template<typename OtherDerived>
    ColVector6d &operator=(const Eigen::MatrixBase<OtherDerived> &other);

    Vector3d_t angular_vector() const;
    Vector3d_t linear_vector() const;

    void set_angular_vector(const Vector3d_t &angular_vector);
    void set_linear_vector(const Vector3d_t &linear_vector);

protected:

private:

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace math {

namespace math {

namespace access {

template<typename Scalar>
    using Vector3d = Eigen::Matrix<Scalar, 3, 1>;
template<typename Scalar>
    using Vector6d = Eigen::Matrix<Scalar, 6, 1>;

template<typename Scalar>
Vector3d<Scalar> AngularVector(const Vector6d<Scalar> &vector6d)
{
    return vector6d.head(3);
}

template<typename Scalar>
Vector3d<Scalar> LinearVector(const Vector6d<Scalar> &vector6d)
{
    return vector6d.tail(3);
}

template<typename Scalar>
void SetAngularVector(
        const Vector3d<Scalar> &vector3d,
        Vector6d<Scalar> *vector6d
        )
{
    vector6d->head(3) = vector3d;
}

template<typename Scalar>
void SetLinearVector(
        const Vector3d<Scalar> &vector3d,
        Vector6d<Scalar> *vector6d
        )
{
    vector6d->tail(3) = vector3d;
}

} // namespace math::access {
} // namespace math

namespace math {

// public func.

template<typename Scalar>
ColVector6d<Scalar>::ColVector6d()
    :Base_t(Base_t::Zero())
{
}

template<typename Scalar>
ColVector6d<Scalar>::ColVector6d(const Vector3d_t &rot_vector, const Vector3d_t &xlt_vector)
{
    access::SetAngularVector(rot_vector, this);
    access::SetLinearVector(xlt_vector, this);
}

template<typename Scalar>
template<typename OtherDerived>
ColVector6d<Scalar>::ColVector6d(const Eigen::MatrixBase<OtherDerived> &other)
    :Base_t(other)
{
}

template<typename Scalar>
ColVector6d<Scalar>::~ColVector6d()
{
}

template<typename Scalar>
template<typename OtherDerived>
ColVector6d<Scalar> &ColVector6d<Scalar>::operator=(const Eigen::MatrixBase<OtherDerived> &other)
{
    this->operator=(other);
    return *this;
}

template<typename Scalar>
typename ColVector6d<Scalar>::Vector3d_t ColVector6d<Scalar>::angular_vector() const
{
    return access::AngularVector(*this);
}

template<typename Scalar>
typename ColVector6d<Scalar>::Vector3d_t ColVector6d<Scalar>::linear_vector() const
{
    return access::LinearVector(*this);
}

template<typename Scalar>
void ColVector6d<Scalar>::set_angular_vector(const Vector3d_t &angular_vector)
{
    access::SetAngularVector(
            angular_vector,
            this
            );
}

template<typename Scalar>
void ColVector6d<Scalar>::set_linear_vector(const Vector3d_t &linear_vector)
{
    access::SetLinearVector(
            linear_vector,
            this
            );
}

// protected func.

// private func.

} // namespace math {

#endif // #ifndef _MATH_COL_VECTOR_6D_H_
