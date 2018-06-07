#ifndef _KINEMATICS_INVERSE_JACOBIAN_IK_SOLVER_H_
#define _KINEMATICS_INVERSE_JACOBIAN_IK_SOLVER_H_

#include "kinematics_def.h"
#include <memory>
#include "boost/optional.hpp"
#include "math_type.h"
#include "acclaim_fwd.h"
#include "kinematics_type.h"

namespace kinematics {

class InverseJacobianIkSolver final
{

public:

    InverseJacobianIkSolver();
    InverseJacobianIkSolver(const InverseJacobianIkSolver &) = delete;
    virtual ~InverseJacobianIkSolver();
    InverseJacobianIkSolver &operator=(const InverseJacobianIkSolver &) = delete;
    /**
     * \brief Configure the related paramters of inverse jacobian IK
     * \param[in] skeleton The skeleton used to compute forward kinematics
     * \param[in] linear_system_solver The solver used to compute inverse Jacobian
     * \param[in] step Linearization step length
     * \param[in] distance_epsilon The desired tolerance of the distance between the target position & end-effector
     * \param[in] max_iteration_num The maximum allowable iteration number to solve IK
     */
    void Configure(
            const std::shared_ptr<acclaim::Skeleton> &skeleton,
            const std::shared_ptr<math::LinearSystemSolver> &linear_system_solver,
            const double step,
            const double distance_epsilon,
            const int32_t max_iteration_num
            );
    /**
     * \brief Solve the inverse Jacobian IK
     * \param[in] target_pos The target position of the end-effector
     * \param[in] start_bone_idx The start index of the bones used to compute IK
     * \param[in] end_bone_idx The end-effecotr index
     * \param[in] original_whole_body_joint_pos6d The reference pose for IK
     * \return The whole body joint positions computed by the IK
     */
    math::Vector6dColl_t Solve(
            const math::Vector3d_t &target_pos,
            const int32_t start_bone_idx,
            const int32_t end_bone_idx,
            const math::Vector6dColl_t &original_whole_body_joint_pos6d
            );
    
protected:
    
private:

    std::shared_ptr<acclaim::Skeleton> skeleton_;
    std::unique_ptr<ForwardSolver> fk_solver_;
    double step_;
    double distance_epsilon_;
    int32_t max_iteration_num_;
    std::shared_ptr<math::LinearSystemSolver> linear_system_solver_;
};

}

#endif // #ifndef _KINEMATICS_INVERSE_JACOBIAN_IK_SOLVER_H_
