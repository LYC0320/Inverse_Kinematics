#ifndef _KINEMATICS_POSE_H_
#define _KINEMATICS_POSE_H_

#include "kinematics_def.h"
#include <memory>
#include "math_type.h"
#include "kinematics_type.h"

namespace kinematics {

class Pose
{

public:

    Pose();
    explicit Pose(
            const math::Vector3d_t &start_pos,
            const math::Vector3d_t &end_pos,
            const math::RotMat3d_t &rotation
            );
    Pose(const Pose &other);
    virtual ~Pose();
    
    Pose &operator=(const Pose &other);

    math::Vector3d_t start_pos() const;
    math::Vector3d_t end_pos() const;
    math::RotMat3d_t rotation() const;

    void set_start_pos(const math::Vector3d_t &start_pos);
    void set_end_pos(const math::Vector3d_t &end_pos);
    void set_rotation(const math::RotMat3d_t &rotation);

    bool is_approx(const Pose &other, const math::Vector3d_t::RealScalar &epsilon) const;

protected:

private:

    std::unique_ptr<math::Vector3d_t> start_pos_;
    std::unique_ptr<math::Vector3d_t> end_pos_;
    std::unique_ptr<math::RotMat3d_t> rotation_;
};

} // namespace kinematics {

#endif // #ifndef _KINEMATICS_POSE_H_
