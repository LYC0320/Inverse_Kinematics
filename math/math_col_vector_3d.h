#ifndef _MATH_COL_VECTOR_3D_H_
#define _MATH_COL_VECTOR_3D_H_

#include "global_def.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <functional>
#undef Success ///< Avoid the conflicts with X11.h
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace math {

template<typename Scalar>
class ColVector3d : public Eigen::Matrix<Scalar, 3, 1>
{
    typedef Eigen::Matrix<Scalar, 3, 1> Base_t;

public:

    typedef typename Base_t::RealScalar RealScalar;

    ColVector3d();
    explicit ColVector3d(
            const Scalar &coord_x,
            const Scalar &coord_y,
            const Scalar &coord_z
            );
    ColVector3d(const Scalar *coord_xyz);
    /**
     * \brief This constructor allows you to construct ColVector3d from Eigen
     * expressions
     */
    template<typename OtherDerived>
    ColVector3d(const Eigen::MatrixBase<OtherDerived> &other);
    virtual ~ColVector3d();
    /**
     * \brief This method allows you to assign Eigen expressions to ColVector3d
     */
    template<typename OtherDerived>
    ColVector3d &operator=(const Eigen::MatrixBase<OtherDerived> &other);

    void set_x(const Scalar &coord_x);
    void set_y(const Scalar &coord_y);
    void set_z(const Scalar &coord_z);
    void set_xyz(const Scalar &coord_x, const Scalar &coord_y, const Scalar &coord_z);
    void set_xyz(const Scalar *coord_xyz);

    ColVector3d ProjectToXyPlane() const;
    ColVector3d ProjectToYzPlane() const;
    ColVector3d ProjectToXzPlane() const;

    ColVector3d ProjectToXyPlane(const Scalar &plane_height) const;
    ColVector3d ProjectToYzPlane(const Scalar &plane_height) const;
    ColVector3d ProjectToXzPlane(const Scalar &plane_height) const;

protected:

private:

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace math {

namespace math {

// public func.

template<typename Scalar>
ColVector3d<Scalar>::ColVector3d()
    :Base_t(Base_t::Zero())
{
}

template<typename Scalar>
ColVector3d<Scalar>::ColVector3d(
        const Scalar &coord_x,
        const Scalar &coord_y,
        const Scalar &coord_z
        )
    :Base_t((Base_t() << coord_x, coord_y, coord_z).finished())
{
}

template<typename Scalar>
ColVector3d<Scalar>::ColVector3d(const Scalar *coord_xyz)
    :Base_t((Base_t() << coord_xyz[0], coord_xyz[1], coord_xyz[2]).finished())
{
}

template<typename Scalar>
template<typename OtherDerived>
ColVector3d<Scalar>::ColVector3d(const Eigen::MatrixBase<OtherDerived> &other)
    :Base_t(other)
{
}

template<typename Scalar>
ColVector3d<Scalar>::~ColVector3d()
{
}

template<typename Scalar>
template<typename OtherDerived>
ColVector3d<Scalar> &ColVector3d<Scalar>::operator=(const Eigen::MatrixBase<OtherDerived> &other)
{
    this->Base_t::operator=(other);
    return *this;
}

template<typename Scalar>
void ColVector3d<Scalar>::set_x(const Scalar &coord_x)
{
    Base_t::operator()(0) = coord_x;
}

template<typename Scalar>
void ColVector3d<Scalar>::set_y(const Scalar &coord_y)
{
    Base_t::operator()(1) = coord_y;
}

template<typename Scalar>
void ColVector3d<Scalar>::set_z(const Scalar &coord_z)
{
    Base_t::operator()(2) = coord_z;
}

template<typename Scalar>
void ColVector3d<Scalar>::set_xyz(const Scalar &coord_x, const Scalar &coord_y, const Scalar &coord_z)
{
    (*this) << coord_x, coord_y, coord_z;
}

template<typename Scalar>
void ColVector3d<Scalar>::set_xyz(const Scalar *coord_xyz)
{
    (*this) << coord_xyz[0], coord_xyz[1], coord_xyz[2];
}

template<typename Scalar>
ColVector3d<Scalar> ColVector3d<Scalar>::ProjectToXyPlane() const
{
    return this->ProjectToXyPlane(Scalar{0.0});
}

template<typename Scalar>
ColVector3d<Scalar> ColVector3d<Scalar>::ProjectToYzPlane() const
{
    this->ProjectToYzPlane(Scalar{0.0});
}

template<typename Scalar>
ColVector3d<Scalar> ColVector3d<Scalar>::ProjectToXzPlane() const
{
    return this->ProjectToXzPlane(Scalar{0.0});
}

template<typename Scalar>
ColVector3d<Scalar> ColVector3d<Scalar>::ProjectToXyPlane(const Scalar &plane_height) const
{
    return ColVector3d<Scalar>(
            this->x(),
            this->y(),
            plane_height
            );
}

template<typename Scalar>
ColVector3d<Scalar> ColVector3d<Scalar>::ProjectToYzPlane(const Scalar &plane_height) const
{
    return ColVector3d<Scalar>(
            plane_height,
            this->y(),
            this->z()
            );
}

template<typename Scalar>
ColVector3d<Scalar> ColVector3d<Scalar>::ProjectToXzPlane(const Scalar &plane_height) const
{
    return ColVector3d<Scalar>(
            this->x(),
            plane_height,
            this->z()
            );
}

// protected func.

// private func.

// non-member func.

template<typename charT, typename traits, typename Scalar>
std::basic_ostream<charT, traits> &operator<<(
        std::basic_ostream<charT, traits> &ostream,
        const ColVector3d<Scalar> &col_vector3d
        )
{
    ostream << col_vector3d.x() << ", "
        << col_vector3d.y() << ", "
        << col_vector3d.z();

    return ostream;
}

} // namespace math {

#endif // #ifndef _MATH_COL_VECTOR_3D_H_
