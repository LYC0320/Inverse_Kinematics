#include "gui_inverse_kinematics_display.h"
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream> 
#include <iomanip>
#include "FL/glut.H"
#include "FL/gl.h"
#include "boost/numeric/conversion/cast.hpp"
#include "console_log.h"
#include "param_config.h"
#include "math_utils.h"
#include "math_pseudoinverse_solver.h"
#include "math_damped_least_squares_solver.h"
#include "acclaim_skeleton.h"
#include "acclaim_motion.h"
#include "geometry_sphere.h"
#include "kinematics_forward_solver.h"
#include "kinematics_pose.h"
#include "kinematics_inverse_jacobian_ik_solver.h"
#include "gui_utils.h"
#include "gui_color.h"
#include "gui_renderer.h"
#include "gui_skeleton_color.h"

namespace gui {

InverseKinematicsDisplay::InverseKinematicsDisplay()
    :frame_idx_(int32_t{0}),
    spot_joint_idx_(int32_t{-1}),
    skeleton_coll_(new SkeletonColl_t),
    motion_coll_(new MotionColl_t),
    fk_solver_coll_(new FkSolverColl_t),
    skeleton_root_offset_coll_(new math::Vector6dColl_t),

    inverse_jacobian_ik_solver_(nullptr),
    ik_trajectory_(new math::Vector3dColl_t),
    ik_trajectory_idx_(0),
    end_effector_trajectory_(new std::deque<math::Vector3d_t>),
    max_end_effector_trajectory_size_(0),
    param_(nullptr),
    renderer_(nullptr),
    skeleton_color_(new SkeletonColor)
{
    skeleton_coll_->reserve(InverseKinematicsDisplay::skeleton_num_hint());
    motion_coll_->reserve(InverseKinematicsDisplay::motion_num_hint());
    fk_solver_coll_->reserve(InverseKinematicsDisplay::skeleton_num_hint());
    skeleton_root_offset_coll_->reserve(InverseKinematicsDisplay::skeleton_num_hint());
}

InverseKinematicsDisplay::~InverseKinematicsDisplay()
{
}

int32_t InverseKinematicsDisplay::spot_joint_idx() const
{
    return spot_joint_idx_;
}

std::shared_ptr<acclaim::Skeleton> InverseKinematicsDisplay::skeleton(const int32_t skeleton_idx) const
{
    return (*skeleton_coll_)[skeleton_idx];
}

std::shared_ptr<acclaim::Motion> InverseKinematicsDisplay::motion(const int32_t motion_idx) const
{
    return (*motion_coll_)[motion_idx];
}

math::Vector6d_t InverseKinematicsDisplay::skeleton_root_offset(const int32_t skeleton_idx) const
{
    return skeleton_root_offset_coll_->at(skeleton_idx);
}

int32_t InverseKinematicsDisplay::skeleton_num() const
{
    return boost::numeric_cast<int32_t>(skeleton_coll_->size());
}

int32_t InverseKinematicsDisplay::motion_num() const
{
    return boost::numeric_cast<int32_t>(motion_coll_->size());
}

int32_t InverseKinematicsDisplay::frame_num(const int32_t motion_idx) const
{
    return motion_coll_->at(motion_idx)->frame_num();
}

int32_t InverseKinematicsDisplay::frame_offset(const int32_t motion_idx) const
{
    return motion_coll_->at(motion_idx)->frame_offset();
}

int32_t InverseKinematicsDisplay::offset_frame_num(const int32_t motion_idx) const
{
    return this->frame_num(motion_idx) - this->frame_offset(motion_idx);
}

Posture InverseKinematicsDisplay::posture(const int32_t motion_idx, const int32_t frame_idx) const
{
    const std::shared_ptr<acclaim::Motion> target_motion = motion_coll_->at(motion_idx);
    return *(target_motion->posture_ptr(target_motion->posture_idx(frame_idx)));
}

int32_t InverseKinematicsDisplay::ik_trajectory_size() const
{
    return boost::numeric_cast<int32_t>(ik_trajectory_->size());
}

void InverseKinematicsDisplay::set_spot_joint_idx(const int32_t spot_joint_idx)
{
    spot_joint_idx_ = spot_joint_idx;
}

void InverseKinematicsDisplay::set_frame_idx(const int32_t frame_idx)
{
    frame_idx_ = frame_idx;
}

void InverseKinematicsDisplay::set_skeleton_root_offset(
        const int32_t skeleton_idx,
        const math::Vector6d_t &root_offset
        )
{
    (*skeleton_root_offset_coll_)[skeleton_idx] = root_offset;
}

void InverseKinematicsDisplay::Configure(
        const std::shared_ptr<param::Config> &param,
        const std::shared_ptr<Renderer> &renderer
        )
{
    param_ = param;
    renderer_ = renderer;

    static const ColorColl_t skeleton_color_coll =
    {
        Color_t(1.0f,  1.0f, 0.1f, 0.9f), // yellow
        Color_t(1.0f, 0.54f, 0.1f, 0.9f), // orange
        Color_t(1.0f,  0.2f, 0.2f, 1.0f), // red
        Color_t(0.2f,  1.0f, 0.2f, 1.0f), // green
        Color_t(0.2f,  0.2f, 1.0f, 1.0f), // blue
    };

    skeleton_color_->Configure(skeleton_color_coll);
}

void InverseKinematicsDisplay::LoadSkeleton(const std::shared_ptr<acclaim::Skeleton> &skeleton)
{
    skeleton_coll_->push_back(skeleton);
    std::shared_ptr<kinematics::ForwardSolver> fk_solver(new kinematics::ForwardSolver);
    fk_solver_coll_->push_back(fk_solver);
    fk_solver_coll_->back()->set_skeleton(skeleton_coll_->back());
    fk_solver_coll_->back()->ConstructArticPath();

    skeleton_root_offset_coll_->push_back(math::Vector6d_t::Zero());
}

void InverseKinematicsDisplay::LoadMotion(const std::shared_ptr<acclaim::Motion> &motion)
{
    motion_coll_->push_back(motion);
}

void InverseKinematicsDisplay::Show()
{
    this->ShowIkTrajectory();

    glPushMatrix();

    for (int32_t skeleton_idx = 0; skeleton_idx < this->skeleton_num(); ++skeleton_idx)
    {
        renderer_->set_translation(
                skeleton_root_offset_coll_->at(skeleton_idx).linear_vector() * acclaim::MoCapScale()
                );
        renderer_->set_rotation(
                skeleton_root_offset_coll_->at(skeleton_idx).angular_vector().x(), math::UnitX()
                );
        renderer_->set_rotation(
                skeleton_root_offset_coll_->at(skeleton_idx).angular_vector().y(), math::UnitY()
                );
        renderer_->set_rotation(
                skeleton_root_offset_coll_->at(skeleton_idx).angular_vector().z(), math::UnitZ()
                );

        this->ShowSkeleton(skeleton_idx);
    } // for (int32_t skeleton_idx = 0; skeleton_idx < this->actorNum(); ++skeleton_idx)

    glPopMatrix();
}

void InverseKinematicsDisplay::Save()
{
    motion_coll_->at(0)->OutputAmcFile(
            "save.amc",
            acclaim::MoCapScale()
            );
}

void InverseKinematicsDisplay::Reset()
{
    skeleton_coll_->clear();
    motion_coll_->clear();
    skeleton_root_offset_coll_->clear();
    fk_solver_coll_->clear();

    this->ProcessIkTrajectory(false);
    inverse_jacobian_ik_solver_.reset();

    skeleton_coll_->reserve(InverseKinematicsDisplay::skeleton_num_hint());
    motion_coll_->reserve(InverseKinematicsDisplay::motion_num_hint());
    skeleton_root_offset_coll_->reserve(InverseKinematicsDisplay::skeleton_num_hint());
}

void InverseKinematicsDisplay::ProcessIkTrajectory(const bool is_active)
{
    if (is_active)
    {
        const int32_t start_bone_idx = param_->value<int32_t>("ik.start_bone_idx");
        const int32_t end_bone_idx = param_->value<int32_t>("ik.end_bone_idx");
        const kinematics::PoseColl_t whole_body_pose_coll
            = fk_solver_coll_->at(0)->ComputeSkeletonPose(0);
        const kinematics::Pose start_bone_pose = whole_body_pose_coll[start_bone_idx];
        const kinematics::Pose end_bone_pose = whole_body_pose_coll[end_bone_idx];

        const math::Vector3d_t start_to_end_direction
            = (end_bone_pose.end_pos() - start_bone_pose.start_pos()).normalized();

        const math::AngleAxis_t plane_rotation(
                math::ToRadian(param_->value<double>("ik.plane_rotation_degree")),
                math::Vector3d_t::UnitX()
                );

        const math::Vector3d_t plane_normal = plane_rotation * math::Vector3d_t::UnitZ();
        const math::ParametrizedLine3d_t center_direction(
                end_bone_pose.end_pos(),
                plane_rotation * math::Vector3d_t::UnitY()
                );

        const math::Vector3d_t center = center_direction.pointAt(
                param_->value<double>("ik.desired_trajectory_radius")
                );

        const double degree_step = param_->value<double>("ik.desired_trajectory_degree_step");
        const int32_t trajectory_size = boost::numeric_cast<int32_t>(ceil(double{360.0} / degree_step));
        const double radian_step = math::ToRadian(degree_step);
        ik_trajectory_->clear();
        ik_trajectory_->reserve(trajectory_size);
        for (int32_t idx = 0; idx < trajectory_size; ++idx)
        {
            const math::AngleAxis_t angle_axis(
                    boost::numeric_cast<double>(idx) * radian_step,
                    plane_normal
                    );
            const math::Vector3d_t trajectory_pos
                = angle_axis * (end_bone_pose.end_pos() - center) + center;
            ik_trajectory_->push_back(trajectory_pos);
        } // for (int32_t idx = 0; idx < trajectory_size; ++idx)

        max_end_effector_trajectory_size_ = param_->value<int32_t>(
                "ik.max_end_effector_trajectory_size"
                );
    }
    else
    {
        ik_trajectory_->clear();
        max_end_effector_trajectory_size_ = 0;
        end_effector_trajectory_->clear();
    }
}

void InverseKinematicsDisplay::ConfigureIkSolver()
{
    inverse_jacobian_ik_solver_.reset(new kinematics::InverseJacobianIkSolver);
    std::shared_ptr<math::LinearSystemSolver> linear_system_solver;
    if (param_->value<std::string>("ik.linear_system_solver")
            == std::string(math::PseudoinverseSolver::tag()))
    {
        linear_system_solver.reset(new math::PseudoinverseSolver);
    }
    else if (param_->value<std::string>("ik.linear_system_solver")
            == std::string(math::DampedLeastSquaresSolver::tag()))
    {
        linear_system_solver.reset(
                new math::DampedLeastSquaresSolver(
                    param_->value<double>("ik.damping_constant")
                    )
                );
    }
    else
    {
        LOGERR << "invalid linear_system_solver" << std::endl;
    }

    static const int32_t kSkeletonIdx = 0;
    inverse_jacobian_ik_solver_->Configure(
            skeleton_coll_->at(kSkeletonIdx),
            linear_system_solver,
            param_->value<double>("ik.step"),
            param_->value<double>("ik.error"),
            param_->value<int32_t>("ik.max_iteration_num")
            );
}

void InverseKinematicsDisplay::SolveIk(const math::Vector3d_t &ik_target)
{
	/*
    if (ik_trajectory_->empty())
    {
        return;
    }
	*/
    const int32_t start_bone_idx = param_->value<int32_t>("ik.start_bone_idx");
    const int32_t end_bone_idx = param_->value<int32_t>("ik.end_bone_idx");
	/*
    const int32_t max_frame_idx = this->ik_trajectory_size() - 1;
    if (frame_idx_ > max_frame_idx)
    {
        frame_idx_ = max_frame_idx;
    }
	*/
    static const int32_t kIkFrameIdx = 0;
    static const int32_t kIkMotionIdx = 0;
    const math::Vector6dColl_t ik_whole_body_joint_pos6d = inverse_jacobian_ik_solver_->Solve(
            //ik_trajectory_->at(frame_idx_),
            ik_target,
			start_bone_idx,
            end_bone_idx,
            motion_coll_->at(kIkMotionIdx)->joint_spatial_pos(kIkFrameIdx)
            );
    motion_coll_->at(kIkMotionIdx)->set_joint_spatial_pos(
            kIkFrameIdx,
            ik_whole_body_joint_pos6d
            );
}

void InverseKinematicsDisplay::CleanFootskate()
{
}

bool InverseKinematicsDisplay::is_empty_motion(const int32_t motion_idx) const
{
    return motion_idx < boost::numeric_cast<int32_t>(motion_coll_->size()) ? FALSE : TRUE;
}

// private func.

void InverseKinematicsDisplay::ShowSkeleton(const int32_t skeleton_idx)
{
    int32_t fk_frame_idx = 0;
    kinematics::PoseColl_t fk_pose_coll;

    if (skeleton_idx == this->motion_num()) // the actor has no corresponding AMC file -> draw T-pose
    {
        std::shared_ptr<acclaim::Motion> default_motion(
                new acclaim::Motion(
                    (*skeleton_coll_)[skeleton_idx],
                    fk_frame_idx + 1
                    )
                );
        fk_solver_coll_->at(skeleton_idx)->set_motion(default_motion);
        fk_pose_coll = fk_solver_coll_->at(skeleton_idx)->ComputeSkeletonPose(fk_frame_idx);
    }
    else
    {
        const int32_t max_frame_idx = motion_coll_->at(skeleton_idx)->frame_num() - 1;
        if (frame_idx_ > max_frame_idx)
        {
            //current frameIdx > maxFrame of the motion file
            fk_frame_idx = max_frame_idx;
        }
        else
        {
            fk_frame_idx = frame_idx_;
        }
        fk_solver_coll_->at(skeleton_idx)->set_motion(motion_coll_->at(skeleton_idx));
        fk_pose_coll = fk_solver_coll_->at(skeleton_idx)->ComputeSkeletonPose(fk_frame_idx);
    }

    if (!ik_trajectory_->empty())
    {
        if (boost::numeric_cast<int32_t>(end_effector_trajectory_->size())
                < max_end_effector_trajectory_size_)
        {
            end_effector_trajectory_->push_back(
                    fk_pose_coll[param_->value<int32_t>("ik.end_bone_idx")].end_pos()
                    );
        }
        else
        {
            end_effector_trajectory_->pop_front();
            end_effector_trajectory_->push_back(
                    fk_pose_coll[param_->value<int32_t>("ik.end_bone_idx")].end_pos()
                    );
        }
    }
    //draw all bones
    math::Vector3d_t bone_start_pos, bone_end_pos;
    for (int32_t bone_idx = 0; bone_idx < skeleton_coll_->at(skeleton_idx)->bone_num(); ++bone_idx)
    {
        bone_start_pos = fk_pose_coll[bone_idx].start_pos();
        bone_end_pos = fk_pose_coll[bone_idx].end_pos();
        
        //draw bone
        renderer_->set_color((*skeleton_color_)(skeleton_idx));
        double bone_radius = param_->value<double>("bone_radius");
        renderer_->DrawCylinder(
                fk_pose_coll[bone_idx].start_pos(),
                fk_pose_coll[bone_idx].end_pos(),
                bone_radius
                );
        math::Vector3d_t axis_x = fk_pose_coll[bone_idx].rotation().col(math::X_id);
        math::Vector3d_t axis_y = fk_pose_coll[bone_idx].rotation().col(math::Y_id);
        math::Vector3d_t axis_z = fk_pose_coll[bone_idx].rotation().col(math::Z_id);		
        
        double axis_scale = param_->value<double>("axis_scale");
        if (bone_idx == spot_joint_idx_)
        {
            renderer_->set_color(gui::Red());
            renderer_->DrawLine(bone_start_pos, math::Vector3d_t(bone_start_pos + axis_scale * axis_x));
            renderer_->set_color(gui::Green());
            renderer_->DrawLine(bone_start_pos, math::Vector3d_t(bone_start_pos + axis_scale * axis_y));
            renderer_->set_color(gui::Blue());
            renderer_->DrawLine(bone_start_pos, math::Vector3d_t(bone_start_pos + axis_scale * axis_z));
        }
    } // for (int32_t bone_idx = 0; bone_idx < skeleton_coll_->at(skeleton_idx)->bone_num(); ++bone_idx)
}

void InverseKinematicsDisplay::ShowIkTrajectory()
{
    if (ik_trajectory_->empty())
    {
        return;
    }

    renderer_->set_line_width(param_->value<double>("ik.desired_trajectory_line_width"));
    renderer_->set_color(param_->value<Color_t>("ik.desired_trajectory_color"));
    renderer_->DrawPolyline(*ik_trajectory_, true);

    renderer_->set_line_width(param_->value<double>("ik.end_effector_trajectory_line_width"));
    renderer_->set_color(param_->value<Color_t>("ik.end_effector_trajectory_color"));

    math::Vector3dColl_t vertex_coll(
            end_effector_trajectory_->begin(),
            end_effector_trajectory_->end()
            );
    renderer_->DrawPolyline(vertex_coll, false);
}

} // namespace gui {
