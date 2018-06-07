#include "kinematics_inverse_jacobian_ik_solver.h"
#include <limits>
#include "console_log.h"
#include "math_utils.h"
#include "math_linear_system_solver.h"
#include "acclaim_skeleton.h"
#include "acclaim_motion.h"
#include "kinematics_forward_solver.h"
#include "kinematics_pose.h"


using namespace math;
using namespace std;

namespace kinematics {

InverseJacobianIkSolver::InverseJacobianIkSolver()
    :skeleton_(nullptr),
    fk_solver_(new ForwardSolver),
    step_(double{0.0}),
    distance_epsilon_(double{0.0}),
    max_iteration_num_(0),
    linear_system_solver_(nullptr)
{
}

InverseJacobianIkSolver::~InverseJacobianIkSolver()
{
}

void InverseJacobianIkSolver::Configure(
        const std::shared_ptr<acclaim::Skeleton> &skeleton,
        const std::shared_ptr<math::LinearSystemSolver> &linear_system_solver,
        const double step,
        const double distance_epsilon,
        const int32_t max_iteration_num
        )
{
    skeleton_ = skeleton;
    fk_solver_->set_skeleton(skeleton_);
    fk_solver_->ConstructArticPath();

    linear_system_solver_ = linear_system_solver;

    step_ = step;
    distance_epsilon_ = distance_epsilon;
    max_iteration_num_ = max_iteration_num;
}

math::Vector6dColl_t InverseJacobianIkSolver::Solve(
	const math::Vector3d_t &target_pos,
	const int32_t start_bone_idx,
	const int32_t end_bone_idx,
	const math::Vector6dColl_t &original_whole_body_joint_pos6d
	)
{//TO DO

	Vector6dColl_t my_whole_body_joint_pos6d = original_whole_body_joint_pos6d;

	int JacobianColNum = 0;
	int32_t tempIdx = end_bone_idx;

	// compute jacobian column number
	while (tempIdx != skeleton_->bone_ptr(start_bone_idx)->parent->idx)
	{
		JacobianColNum += skeleton_->bone_ptr(tempIdx)->dof;

		tempIdx = skeleton_->bone_ptr(tempIdx)->parent->idx;
	}

	for (int iterationCount = 0; iterationCount < max_iteration_num_; iterationCount++)
	{
		PoseColl_t temp;
		temp = fk_solver_->ComputeSkeletonPose(my_whole_body_joint_pos6d);

		Vector3d_t V = target_pos - temp[end_bone_idx].end_pos();

		if (V.norm() > distance_epsilon_)
		{
			// initial
			tempIdx = end_bone_idx;

			int  dofIdx = 0;

			MatrixN_t Jacobian(3, JacobianColNum);
			
			// compute Jacobian
			while (tempIdx != skeleton_->bone_ptr(start_bone_idx)->parent->idx)
			{
				Vector3d_t pmri = temp[end_bone_idx].end_pos() - temp[tempIdx].start_pos();

				bool x = false, y = false, z = false;

				for (int colIdx = 0; colIdx < skeleton_->bone_ptr(tempIdx)->dof;)
				{
					if (skeleton_->bone_ptr(tempIdx)->dofx && !x)
					{
						Jacobian.col(colIdx + dofIdx) = temp[tempIdx].rotation().col(0).cross(pmri);
						colIdx++;
						x = true;
					}
					else if (skeleton_->bone_ptr(tempIdx)->dofy && !y)
					{
						Jacobian.col(colIdx + dofIdx) = temp[tempIdx].rotation().col(1).cross(pmri);
						colIdx++;
						y = true;
					}
					else if (skeleton_->bone_ptr(tempIdx)->dofz && !z)
					{
						Jacobian.col(colIdx + dofIdx) = temp[tempIdx].rotation().col(2).cross(pmri);
						colIdx++;
						z = true;
					}
				}

				dofIdx += skeleton_->bone_ptr(tempIdx)->dof;
				tempIdx = skeleton_->bone_ptr(tempIdx)->parent->idx;
			}

			// initial
			tempIdx = end_bone_idx;
			dofIdx = 0;

			VectorNd_t deltaSita(Jacobian.cols());
			deltaSita = linear_system_solver_->Solve(Jacobian, V);

			while (tempIdx != skeleton_->bone_ptr(start_bone_idx)->parent->idx)
			{
				Vector3d_t angularVector = my_whole_body_joint_pos6d[tempIdx].angular_vector();

				bool x = false, y = false, z = false;

				for (int colIdx = 0; colIdx < skeleton_->bone_ptr(tempIdx)->dof;)
				{
					if (skeleton_->bone_ptr(tempIdx)->dofx && !x)
					{
						angularVector[0] += deltaSita[0 + dofIdx] * step_;
						colIdx++;
						x = true;
					}
					else if (skeleton_->bone_ptr(tempIdx)->dofy && !y)
					{
						angularVector[1] += deltaSita[1 + dofIdx] * step_;
						colIdx++;
						y = true;
					}
					else if (skeleton_->bone_ptr(tempIdx)->dofz && !z)
					{
						angularVector[2] += deltaSita[2 + dofIdx] * step_;
						colIdx++;
						z = true;
					}
				}

				my_whole_body_joint_pos6d[tempIdx].set_angular_vector(angularVector);

				dofIdx += skeleton_->bone_ptr(tempIdx)->dof;
				tempIdx = skeleton_->bone_ptr(tempIdx)->parent->idx;
			}
		}
		else
		{
			return my_whole_body_joint_pos6d;
		}

	}
	return my_whole_body_joint_pos6d;
}

} // namespace kinematics {

