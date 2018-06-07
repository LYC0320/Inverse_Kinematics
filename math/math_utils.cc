#include "math_utils.h"
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <iterator>
#include <numeric>
#include "boost/numeric/conversion/cast.hpp"
#include "console_log.h"
#include "gl_utils.h"

namespace math {
namespace inner {

template<typename Scalar>
const Scalar Epsilon()
{
    return Scalar{1.0e-6};
}

} // namespace math::inner {
} // namespace math {

namespace math {

template<typename Scalar>
Scalar Pi()
{
    return boost::numeric_cast<Scalar>(3.14159265358979323846264338327950288419716939937510);
}
template double Pi<double>();
template float Pi<float>();

template<typename Scalar>
Scalar ToRadian()
{
    static const Scalar kCoef = Pi<Scalar>() / Scalar{180.0};
    return kCoef;
}
template double ToRadian<double>();
template float ToRadian<float>();

template<typename Scalar>
Scalar ToDegree()
{
    static const Scalar kCoef = Scalar{180.0} / Pi<Scalar>();
    return kCoef;
}
template double ToDegree<double>();
template float ToDegree<float>();

template<typename Scalar>
Scalar ToRadian(const Scalar &degree)
{
    return degree * ToRadian<Scalar>();
}
template double ToRadian<double>(const double &);
template float ToRadian<float>(const float &);

template<typename Scalar>
Scalar ToDegree(const Scalar &radian)
{
    return radian * ToDegree<Scalar>();
}
template double ToDegree<double>(const double &);
template float ToDegree<float>(const float &);

Vector3d_t ToRadian(const Vector3d_t &degree3d)
{
    return Vector3d_t(
            math::ToRadian<Vector3d_t::RealScalar>(degree3d.x()),
            math::ToRadian<Vector3d_t::RealScalar>(degree3d.y()),
            math::ToRadian<Vector3d_t::RealScalar>(degree3d.z())
            );
}

Vector3d_t ToDegree(const Vector3d_t &radian3d)
{
    return Vector3d_t(
            math::ToDegree<Vector3d_t::RealScalar>(radian3d.x()),
            math::ToDegree<Vector3d_t::RealScalar>(radian3d.y()),
            math::ToDegree<Vector3d_t::RealScalar>(radian3d.z())
            );
}

Vector6d_t ToRadian(const Vector6d_t &degree6d)
{
    Vector6d_t radian6d;
    radian6d.set_angular_vector(
            math::ToRadian(degree6d.angular_vector())
            );
    radian6d.set_linear_vector(degree6d.linear_vector());

    return radian6d;
}

Vector6d_t ToDegree(const Vector6d_t &radian6d)
{
    Vector6d_t degree6d;
    degree6d.set_angular_vector(
            math::ToDegree(radian6d.angular_vector())
            );
    degree6d.set_linear_vector(radian6d.linear_vector());

    return degree6d;
}

Vector3dColl_t ToRadian(const Vector3dColl_t &degree3d_coll)
{
    Vector3dColl_t radian3d_coll;
    radian3d_coll.reserve(degree3d_coll.size());
    std::transform(
            degree3d_coll.begin(),
            degree3d_coll.end(),
            std::back_inserter(radian3d_coll),
            [&](const Vector3d_t &degree3d)
            {return math::ToRadian(degree3d);}
            );

    return radian3d_coll;
}

Vector3dColl_t ToDegree(const Vector3dColl_t &radian3d_coll)
{
    Vector3dColl_t degree3d_coll;
    degree3d_coll.reserve(radian3d_coll.size());
    std::transform(
            radian3d_coll.begin(),
            radian3d_coll.end(),
            std::back_inserter(degree3d_coll),
            [&](const Vector3d_t &radian3d)
            {return math::ToDegree(radian3d);}
            );

    return degree3d_coll;
}

Vector6dColl_t ToRadian(const Vector6dColl_t &degree6d_coll)
{
    Vector6dColl_t radian6d_coll;
    radian6d_coll.reserve(degree6d_coll.size());
    std::transform(
            degree6d_coll.begin(),
            degree6d_coll.end(),
            std::back_inserter(radian6d_coll),
            [](const Vector6d_t &degree6d)
            {return math::ToRadian(degree6d);}
            );

    return radian6d_coll;
}

Vector6dColl_t ToDegree(const Vector6dColl_t &radian6d_coll)
{
    Vector6dColl_t degree6d_coll;
    degree6d_coll.reserve(radian6d_coll.size());

    std::transform(
            radian6d_coll.begin(),
            radian6d_coll.end(),
            std::back_inserter(degree6d_coll),
            [&](const Vector6d_t &radian6d)
            {return math::ToDegree(radian6d);}
            );

    return degree6d_coll;
}

Vector3d_t UnitX()
{
    return Vector3d_t::UnitX();
}

Vector3d_t UnitY()
{
    return Vector3d_t::UnitY();
}

Vector3d_t UnitZ()
{
    return Vector3d_t::UnitZ();
}


void ToArray3d(const Vector3d_t &vector3d,  double *array3d)
{
    array3d[0] = vector3d.x();
    array3d[1] = vector3d.y();
    array3d[2] = vector3d.z();
}

double ComputeAngle(const Vector3d_t &vector1st, const Vector3d_t &vector2nd, const Vector3d_t &axis)
{
	double dot_prod = vector1st.dot(vector2nd);
	double rotAxis_length = axis.norm();
	double theta = atan2(rotAxis_length, dot_prod);
	return theta;    
}

/**** Matrix Function ****/

RotMat3d_t ComputeRotMatX(const double radian)
{
    return AngleAxis_t(radian, Vector3d_t::UnitX()).toRotationMatrix();
}

RotMat3d_t ComputeRotMatY(const double radian)
{
    return AngleAxis_t(radian, Vector3d_t::UnitY()).toRotationMatrix();
}

RotMat3d_t ComputeRotMatZ(const double radian)
{
    return AngleAxis_t(radian, Vector3d_t::UnitZ()).toRotationMatrix();
}

RotMat3d_t ComputeRotMatXyz(const double radian_x, const double radian_y, const double radian_z)
{
    return ComputeRotMatZ(radian_z) * ComputeRotMatY(radian_y) * ComputeRotMatX(radian_x);
}

RotMat3d_t ComputeRotMatZyx(const double radian_x, const double radian_y, const double radian_z)
{
    return ComputeRotMatX(radian_x) * ComputeRotMatY(radian_y) * ComputeRotMatZ(radian_z);
}

RotMat3d_t ComputeRotMatXyz(const Vector3d_t &radian3d)
{
    return ComputeRotMatXyz(radian3d.x(), radian3d.y(), radian3d.z());
}

RotMat3d_t ComputeRotMatZyx(const Vector3d_t &radian3d)
{
    return ComputeRotMatZyx(radian3d.x(), radian3d.y(), radian3d.z());
}

RotMat3d_t ComputeRotMat(const Quaternion_t &quat)
{
    return RotMat3d_t(quat);
}

Vector3d_t ComputeEulerAngleXyz(const RotMat3d_t &rot_mat)
{
    typedef RotMat3d_t::RealScalar Scalar_t;
    if (rot_mat(2, 0) > (Scalar_t{1.0} - inner::Epsilon<Scalar_t>()))
    {
        Scalar_t angle_x = atan2(rot_mat(0, 1), rot_mat(0, 2));
        Scalar_t angle_y = -math::HalfPi<Scalar_t>();
        Scalar_t angle_z = Scalar_t{0.0};

        return Vector3d_t(angle_x, angle_y, angle_z);
    }

    if (rot_mat(2, 0) < -(Scalar_t{1.0} - inner::Epsilon<Scalar_t>()))
    {
        Scalar_t angle_x = atan2(rot_mat(0, 1), rot_mat(0, 2));
        Scalar_t angle_y = math::HalfPi<Scalar_t>();
        Scalar_t angle_z = Scalar_t{0.0};

        return Vector3d_t(angle_x, angle_y, angle_z);
    }

    Scalar_t angle_x = atan2(rot_mat(2, 1), rot_mat(2, 2));
    Scalar_t angle_y = -asin(rot_mat(2, 0));
    Scalar_t angle_z = atan2(rot_mat(1, 0), rot_mat(0, 0));

    return Vector3d_t(angle_x, angle_y, angle_z);
}

Vector3d_t ComputeEulerAngleZyx(const RotMat3d_t &rot_mat)
{
    typedef RotMat3d_t::RealScalar Scalar_t;
    if (rot_mat(0, 2) > (Scalar_t{1.0} - inner::Epsilon<Scalar_t>()))
    {
        Scalar_t angle_z = atan2(rot_mat(1, 0), rot_mat(1, 1));
        Scalar_t angle_y = math::HalfPi<Scalar_t>();
        Scalar_t angle_x = Scalar_t{0.0};

        return Vector3d_t(angle_z, angle_y, angle_x);
    }

    if(rot_mat(0, 2) < -(Scalar_t{1.0} - inner::Epsilon<Scalar_t>()))
    {
        Scalar_t angle_z = atan2(rot_mat(1, 0), rot_mat(1, 1));
        Scalar_t angle_y = -math::HalfPi<Scalar_t>();
        Scalar_t angle_x = Scalar_t{0.0};

        return Vector3d_t(angle_z, angle_y, angle_x);
    }

    Scalar_t angle_z = -atan2(rot_mat(0, 1), rot_mat(0, 0));
    Scalar_t angle_y = asin(rot_mat(0, 2));
    Scalar_t angle_x = -atan2(rot_mat(1, 2), rot_mat(2, 2));

    return Vector3d_t(angle_z, angle_y, angle_x);	// order of return is the order of input
}

RotMat3d_t ToRotMat(const double mat4[4][4])
{
    RotMat3d_t output_rotmat;

    for (int32_t row_idx = 0; row_idx < output_rotmat.rows(); ++row_idx)
    {
        for (int32_t col_idx = 0; col_idx < output_rotmat.cols(); ++col_idx)
        {
            output_rotmat(row_idx, col_idx) = mat4[row_idx][col_idx];
        }
    }
    return output_rotmat;
}

void ToMatrix4(const RotMat3d_t &rot_mat, double mat4[4][4])
{
    memset(mat4, 0, sizeof(mat4[0][0]) * 4 * 4);
    mat4[3][3] = 1.0;

    for (int32_t row_idx = 0; row_idx < rot_mat.rows(); ++row_idx)
    {
        for (int32_t col_idx = 0; col_idx < rot_mat.cols(); ++col_idx)
        {
             mat4[row_idx][col_idx] = rot_mat(row_idx, col_idx);
        }
    }    
}

void ToGlMatrix4(
        const RotMat3d_t &rot_mat,
        const Vector3d_t &pos,
        double mat[glmat4::Size]
        )
{
    glmat4::SetIdentity(mat);
    for (int row_idx = 0; row_idx < rot_mat.rows(); ++row_idx)
    {
        for (int col_idx = 0; col_idx < rot_mat.cols(); ++col_idx)
        {
            int mat_idx = row_idx + col_idx * glmat4::Rows();
            mat[mat_idx] = rot_mat(row_idx, col_idx);
        }
    }

    mat[glmat4::idx::XltX()] = pos.x();
    mat[glmat4::idx::XltY()] = pos.y();
    mat[glmat4::idx::XltZ()] = pos.z();

    Eigen::Affine3d affine_xfm;
    affine_xfm.linear() = rot_mat;
    affine_xfm.translation() = pos;

    for (int idx = 0; idx < glmat4::Size; ++idx)
    {
        static const double kEpsilon = 1.0e-6;
        double diff = fabs(mat[idx] - affine_xfm.data()[idx]);
        if (diff > kEpsilon)
        {
            LOGERR << "Invalid conversion of the homogeneous transformation" << std::endl;
            for (int array_idx = 0; array_idx < glmat4::Size; ++array_idx)
            {
                std::cout << std::setprecision(4) << std::fixed
                    << "idx: " << array_idx << ", "
                    << "data: " << affine_xfm.data()[array_idx] << ", "
                    << "glmat: " << mat[array_idx] << std::endl;
            }

            assert(FALSE);
        }
    }
}

/**** Quaternion Function ****/

Quaternion_t ComputeQuaternionXyz(const double radian_x, const double radian_y, const double radian_z)
{
    RotMat3d_t rotmat_xyz = ComputeRotMatXyz(radian_x, radian_y, radian_z);
    return Quaternion_t(rotmat_xyz);
}

Quaternion_t ComputeQuaternionZyx(const double radian_x, const double radian_y, const double radian_z)
{
    RotMat3d_t rotmat_zyx = ComputeRotMatZyx(radian_x, radian_y, radian_z);
    return Quaternion_t(rotmat_zyx);
}

Quaternion_t ComputeAverageQuaternion(const QuaternionColl_t &quat_coll)
{
    typedef Quaternion_t::RealScalar Scalar_t;
    auto to_vector4d = [](const Quaternion_t &input_quat)
    {
        return Vector4d_t(input_quat.x(), input_quat.y(), input_quat.z(), input_quat.w());
    };
    auto to_quaternion = [](const Vector4d_t &input_vector4d)
    {
        return Quaternion_t(input_vector4d.w(), input_vector4d.x(), input_vector4d.y(), input_vector4d.z());
    };
    auto is_closed = [](const Quaternion_t &ref_quat, const Quaternion_t &trg_quat)
    {
        return ref_quat.dot(trg_quat) >= 0 ? TRUE : FALSE;
    };

    Vector4d_t accumulate_quat_vector = Vector4d_t::Zero();

    const int quat_num = static_cast<int>(quat_coll.size());
    for (auto quat = quat_coll.begin(); quat != (quat_coll.begin() + quat_num); ++quat)
    {
        if (!is_closed(quat_coll.front(), *quat))
        {
            accumulate_quat_vector += to_vector4d(quat->inverse());
            LOGMSG << "Not closed quaternion" << std::endl;
        }
        else
        {
            accumulate_quat_vector += to_vector4d(*quat);
        }
    }

    Scalar_t reciprocal_size = static_cast<Scalar_t>(1.0) / static_cast<Scalar_t>(quat_num);
    Vector4d_t average_quat_vector = accumulate_quat_vector * reciprocal_size;
    average_quat_vector.normalize();

    return to_quaternion(average_quat_vector);
}

Vector3d_t Lerp(
    const Vector3d_t &start_vector,
    const Vector3d_t &end_vector,
    const Vector3d_t::RealScalar &step_ratio
    )
{
    static const Vector3d_t::RealScalar kUnitStepRatio = Vector3d_t::RealScalar{1.0};
    Vector3d_t lerp_vector = (kUnitStepRatio - step_ratio) * start_vector + step_ratio * end_vector;
    return lerp_vector;
}

Quaternion_t Slerp(
    const Quaternion_t &start_quat,
    const Quaternion_t &end_quat,
    const Quaternion_t::RealScalar &step_ratio
    )
{
    Quaternion_t slerp_quat = start_quat.slerp(
        step_ratio,
        end_quat
        );
    return slerp_quat;
}


bool IsApproxZero(
        const MatrixN_t &input_mat,
        const MatrixN_t::RealScalar &epsilon
        )
{
    return input_mat.isZero(epsilon);
}

bool IsApprox(
        const MatrixN_t &ref,
        const MatrixN_t &trg,
        const MatrixN_t::RealScalar &epsilon
        )
{
    MatrixN_t diff_mat = ref - trg;
    return diff_mat.isZero(epsilon);
}

template<typename Scalar>
bool IsApprox(
        const Scalar &ref,
        const Scalar &trg,
        const Scalar &epsilon
        )
{
    double diff = fabs(static_cast<double>(ref) - static_cast<double>(trg));
    return diff < static_cast<double>(epsilon) ? TRUE : FALSE;
}
template bool IsApprox<double>(const double &, const double &, const double &);
template bool IsApprox<float>(const float &, const float &, const float &);

bool IsNan(const MatrixN_t &mat)
{
    typedef MatrixN_t::Index Idx_t;
    for (Idx_t row_idx = Idx_t{0}; row_idx < mat.rows(); ++row_idx)
    {
        for (Idx_t col_idx = Idx_t{0}; col_idx < mat.cols(); ++col_idx)
        {
            if (std::isnan(mat(row_idx, col_idx)))
            {
                return TRUE;
            }
        }
    }

    return FALSE;
}

RotMat3d_t ComputeRelativeRotMat(
        const Vector3d_t &src_vector,
        const Vector3d_t &end_vector
        )
{
    const Vector3d_t unit_src_vector = src_vector.normalized();
    const Vector3d_t unit_end_vector = end_vector.normalized();
    Quaternion_t targ_quat;
    targ_quat.setFromTwoVectors(unit_src_vector, unit_end_vector);

    {
        Vector3d_t targ_end_vector = targ_quat.toRotationMatrix() * unit_src_vector;
        if (!math::IsApprox(unit_end_vector, targ_end_vector, inner::Epsilon<Vector3d_t::RealScalar>()))
        {
            LOGWARN << std::setprecision(5) << std::fixed << '\n'
                << "Cannot convert src_vector to end_vector by rotation matrix" << '\n'
                << "src_vector: " << src_vector.transpose() << '\n'
                << "end_vector: " << end_vector.transpose() << '\n'
                << std::endl;
            assert(FALSE);
        }
    }

    return targ_quat.toRotationMatrix();
}

}   //namespace math
