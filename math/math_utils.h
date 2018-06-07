#ifndef _MATH_UTILS_H_
#define _MATH_UTILS_H_

#include "math_type.h"
#include "gl_utils.h"
#include "math_col_vector_3d.h"

namespace math {

enum XYZ_idx{
    X_id = 0,
    Y_id, // = 1
    Z_id, // = 2
    Num   // = 3
};

/**
 * \brief
 * \param[in]
 * \return
 */
template<typename Scalar>
const Scalar Square(const Scalar &value)
{return value * value;}
/**
 * \brief
 * \param[in]
 * \return
 */
template<typename Scalar>
const Scalar Cube(const Scalar &value)
{return value * value * value;}
/**
 * \brief
 */
template<typename Scalar>
const Scalar Half(const Scalar &value)
{return Scalar{0.5} * value;}
/**
 * \brief
 * \return
 */
template<typename Scalar>
Scalar Pi();
/**
 * \brief
 */
template<typename Scalar>
const Scalar HalfPi()
{return Scalar{0.5} * math::Pi<Scalar>();}
/**
 * \brief
 * \return
 */
template<typename Scalar>
Scalar ToRadian();
/**
 * \brief
 * \return
 */
template<typename Scalar>
Scalar ToDegree();
/**
 * \brief
 * \param[in]
 * \return
 */
template<typename Scalar>
Scalar ToRadian(const Scalar &degree);
/**
 * \brief
 * \param[in]
 * \return
 */
template<typename Scalar>
Scalar ToDegree(const Scalar &radian);
/**
 * \brief
 * \param[in]
 * \return
 */
Vector3d_t ToRadian(const Vector3d_t &degree3d);
/**
 * \brief
 * \param[in]
 * \return
 */
Vector3d_t ToDegree(const Vector3d_t &radian3d);
/**
 * \brief
 */
Vector6d_t ToRadian(const Vector6d_t &degree6d);
/**
 * \brief
 */
Vector6d_t ToDegree(const Vector6d_t &radian6d);
/**
 * \brief
 * \param[in]
 * \return
 */
Vector3dColl_t ToRadian(const Vector3dColl_t &degree3d_coll);
/**
 * \brief
 * \param[in]
 * \return
 */
Vector3dColl_t ToDegree(const Vector3dColl_t &radian3d_coll);
/**
 * \brief
 */
Vector6dColl_t ToRadian(const Vector6dColl_t &degree6d_coll);
/**
 * \brief
 */
Vector6dColl_t ToDegree(const Vector6dColl_t &radian6d_coll);
/**
 * \brief
 * \return
 */
Vector3d_t UnitX();
/**
 * \brief
 * \return
 */
Vector3d_t UnitY();
/**
 * \brief
 * \return
 */
Vector3d_t UnitZ();
/**
 * \brief Set array from ColVector3d
 * \param[in] vector3d
 * \param[out] array3d
 */
void ToArray3d(const Vector3d_t &vector3d,  double *array3d);
/**
 * \brief compute the angle from vector v1 to v2 around the axis
 * \param[in] vector1st
 * \param[in] vector2nd
 * \param[in] axis
 */
double ComputeAngle(const Vector3d_t &vector1st, const Vector3d_t &vector2nd, const Vector3d_t &axis);
/**
 * \brief Calculate the rotation matrix around unit x-axis, y-axis or z-axis
 * \param[in] radian
 */
RotMat3d_t ComputeRotMatX(const double radian);
RotMat3d_t ComputeRotMatY(const double radian);
RotMat3d_t ComputeRotMatZ(const double radian);
/**
 * \brief Calculate rotation matrix, using the order X-Y-Z or Z-Y-X, according to the given euler angles
 * \param[in] radian_x
 * \param[in] radian_y
 * \param[in] radian_z
 * \return
 */
RotMat3d_t ComputeRotMatXyz(const double radian_x, const double radian_y, const double radian_z);
/**
 * \brief
 * \param[in]
 * \param[in]
 * \param[in]
 * \return
 */
RotMat3d_t ComputeRotMatZyx(const double radian_x, const double radian_y, const double radian_z);
/**
 * \brief
 * \param[in] radian3d
 * \return
 */
RotMat3d_t ComputeRotMatXyz(const Vector3d_t &radian3d);
/**
 * \brief
 * \param[in] radian3d
 * \return
 */
RotMat3d_t ComputeRotMatZyx(const Vector3d_t &radian3d);
/**
* \brief Conver the quaternion to rotation matrix
* \param[in] quat The target quaternion
* \return The rotation matrix 
*/
RotMat3d_t ComputeRotMat(const Quaternion_t &quat);
/**
 * \brief Convert the rotation matrix to the Euler angles by the order XYZ
 * \param[in] rot_mat The target rotation matrix
 * \return The Euler angles in the order XYZ, i.e., Vector3d_t(AngX, AngY, AngZ)
 */
Vector3d_t ComputeEulerAngleXyz(const RotMat3d_t &rot_mat);
/**
 * \brief Conver the rotation matrix to the Euler angles by the order ZYX
 * \param[in] rot_mat The target rotation matrix
 * \return The Euler angles in the order ZYX, i.e., Vector3d_t(AngZ, AngY, AngX);
 */
Vector3d_t ComputeEulerAngleZyx(const RotMat3d_t &rot_mat);
/**
 * \brief Set rotation matrix from 4x4 array
 * \param[in] mat4
 */
RotMat3d_t ToRotMat(const double mat4[4][4]);
/**
 * \brief Set 4x4 array (affine matrix) from rotation matrix
 */
void ToMatrix4(const RotMat3d_t &rot_mat, double mat4[4][4]);
/**
 * \brief
 * \param[in]  rot_mat
 * \param[in]  pos
 * \param[out] mat
 */
void ToGlMatrix4(
        const RotMat3d_t &rot_mat,
        const Vector3d_t &pos,
        double mat[glmat4::Size]
        );
/**
 * \brief Calc. the quaternion according to the given angles by the axis order
 * X->Y->Z
*/
Quaternion_t ComputeQuaternionXyz(const double radian_x, const double radian_y, const double radian_z);
/**
 * \brief
 * \param[in]
 * \param[in]
 * \param[in]
 * \return
 */
Quaternion_t ComputeQuaternionZyx(const double radian_x, const double radian_y, const double radian_z);
/**
 * \brief
 * \param[in]
 * \return
 */
Quaternion_t ComputeAverageQuaternion(const QuaternionColl_t &quat_coll);
/**
 * \brief Linear interpolation for Vector
 * \param[in] start_vector
 * \param[in] end_vector
 * \param[in] step_ratio
 * \return The interpoalted vector,
 * i.e., (1.0 - step_ratio) * start_vector + step_ratio * end_vector
 */
Vector3d_t Lerp(
    const Vector3d_t &start_vector,
    const Vector3d_t &end_vector,
    const Vector3d_t::RealScalar &step_ratio
    ); 
/**
 * \brief Linear interpolation for quaternion
 * \param[in] start_quat
 * \param[in] end_quat
 * \param[in] step_ratio
 * \return (1.0 - step_ratio) * start_quat + step_ratio * end_quat
 */
Quaternion_t Slerp(
    const Quaternion_t &start_quat,
    const Quaternion_t &end_quat,
    const Quaternion_t::RealScalar &step_ratio
    );
/**
 * \brief Test if the input matrix is a zero matrix approximately, according to a given allowable
 * epsilon
 * \param[in] input_mat The input matrix
 * \param[in] epsilon   The allowable deviation from zero
 * \retval TRUE The input matrix is approximately a zero matrix, i.e., the absolute value of each
 * element in the input matrix is smaller than the given epsilon
 * \retval FALSE The input matrix is NOT a zero matrix, i.e., at least one element's absolute value
 * of the input matrix is greater than the given epsilon
 */
bool IsApproxZero(
        const MatrixN_t &input_mat,
        const MatrixN_t::RealScalar &epsilon
        );
/**
 * \brief Test if two matrices are approximately equal, according to a given allowable epsilon
 * \param[in] ref     The reference matrix
 * \param[in] trg     The target matrix
 * \param[in] epsilon The allowable error of differences
 * \retval TRUE Both matrices are approximately equal, i.e., the differences of each elements are
 * smaller than epsilon
 * \retval FALSE Both matrices are NOT equal, i.e., at least one element of the differences is
 * greater than epsilon
 */
bool IsApprox(
        const MatrixN_t &ref,
        const MatrixN_t &trg,
        const MatrixN_t::RealScalar &epsilon
        );
/**
 * \brief
 * \param[in]
 * \param[in]
 * \param[in]
 * \return
 */
template<typename Scalar>
bool IsApprox(
        const Scalar &ref,
        const Scalar &trg,
        const Scalar &epsilon
        );
/**
 * \brief
 * \param[in]
 * \return
 */
bool IsNan(const MatrixN_t &mat);
/**
 * \brief Compute the rotation matrix to transform src_vector to end_vector, i.e.,
 * end_vector = relativeRotMat * src_vector
 * \param[in] src_vector The source vector
 * \param[in] end_vector The end vector
 * \return The rotation matrix transforming src_vector to end_vector
 */
RotMat3d_t ComputeRelativeRotMat(
        const Vector3d_t &src_vector,
        const Vector3d_t &end_vector
        );

} // namespace math

#endif // #ifndef _MATH_UTILS_H_
