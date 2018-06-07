#ifndef _HELPER_FORWARD_KINEMATICS_H_
#define _HELPER_FORWARD_KINEMATICS_H_

#include "helper_def.h"
#include <memory>
#include "boost/optional.hpp"
#include "math_type.h"
#include "acclaim_fwd.h"
#include "kinematics_type.h"

namespace helper {

class ForwardKinematics final
{

public:

    ForwardKinematics();
    ForwardKinematics(const ForwardKinematics &) = delete;
    virtual ~ForwardKinematics();
    ForwardKinematics &operator=(const ForwardKinematics &) = delete;

    std::shared_ptr<acclaim::Skeleton> skeleton() const;
    std::shared_ptr<acclaim::Motion> motion() const;

    void set_skeleton(const std::shared_ptr<acclaim::Skeleton> &skeleton);
    void set_motion(const std::shared_ptr<acclaim::Motion> &motion);

    void ConstructArticPath();

    kinematics::PoseColl_t ComputeSkeletonPose(const int32_t frame_idx);
    kinematics::PoseColl_t ComputeSkeletonPose(
            const math::Vector6dColl_t &joint_spaital_pos
            );

protected:

private:

    kinematics::ArticIdxColl_t EvalRootToTargetBoneArticPath(
            const int32_t target_body_idx
            ) const;
    math::HomoXfm3d_t ComputeParentJointGlobalXfm(
            const int32_t body_idx,
            const math::Vector6dColl_t &target_joint_pos6d
            );

    std::shared_ptr<acclaim::Skeleton> skeleton_;
    std::shared_ptr<acclaim::Motion> motion_;
    std::unique_ptr<kinematics::ArticIdxColl_t> artic_path_;
};

} // namespace helper {

#endif // #ifndef _HELPER_FORWARD_KINEMATICS_H_
