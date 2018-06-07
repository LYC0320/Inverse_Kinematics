#ifndef _KINEMATICS_FORWARD_SOLVER_H_
#define _KINEMATICS_FORWARD_SOLVER_H_

#include "kinematics_def.h"
#include <memory>
#include "boost/optional.hpp"
#include "math_type.h"
#include "acclaim_fwd.h"
#include "helper_fwd.h"
#include "kinematics_type.h"

namespace kinematics {

class ForwardSolver final
{

public:

    ForwardSolver();
    ForwardSolver(const ForwardSolver &) = delete;
    virtual ~ForwardSolver();
    ForwardSolver &operator=(const ForwardSolver &) = delete;

    std::shared_ptr<acclaim::Skeleton> skeleton() const;
    std::shared_ptr<acclaim::Motion> motion() const;

    void set_skeleton(const std::shared_ptr<acclaim::Skeleton> &skeleton);
    void set_motion(const std::shared_ptr<acclaim::Motion> &motion);

    void ConstructArticPath();

    PoseColl_t ComputeSkeletonPose(const int32_t frame_idx);
    PoseColl_t ComputeSkeletonPose(const math::Vector6dColl_t &joint_spaital_pos);

protected:

private:

    std::shared_ptr<acclaim::Skeleton> skeleton_;
    std::shared_ptr<acclaim::Motion> motion_;
    std::unique_ptr<ArticIdxColl_t> artic_path_;
    std::unique_ptr<helper::ForwardKinematics> helper_fk_;
};

} // namespace kinematics {

#endif // #ifndef _KINEMATICS_FORWARD_SOLVER_H_
