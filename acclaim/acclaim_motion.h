/*
      motion.h
 
	  Motion class  

	  1. read an AMC file and store it in a sequence of state vector 
	  2. write an AMC file
	  3. export to a mrdplot format for plotting the trajectories
   
      You can add more motion data processing functions in this class. 

      Revision 1 - Steve Lin, Jan. 14, 2002
 	  Revision 2 - Alla and Kiran, Jan 18, 2002
*/

#ifndef _ACCLAIM_MOTION_H_
#define _ACCLAIM_MOTION_H_

#include <memory>
#include <vector>
#include "math_type.h"
#include "posture.h"
#include "acclaim_skeleton.h"

namespace acclaim {

class Motion final
{

public:
    static const double frequency()
    {return double{120.0};}
    static const double time_step()
    {return double{1.0} / double{120.0};}
    /**
     * \brief
     * \param[in] skeleton
     * \param[in] frame_num
     */
    Motion(const std::shared_ptr<Skeleton> &skeleton, const int32_t frame_num);
    /**
     * \brief
     * \param[in]
     * \param[in]
     * \param[in]
     */
    Motion(
            const std::string &amc_file_name,
            const double skeleton_scale,
            const std::shared_ptr<Skeleton> &skeleton
            );
    Motion(const Motion &) = delete;
    /**
     * \brief
     */
    virtual ~Motion();
    Motion &operator=(const Motion &) = delete;
    /**
     * \brief Get the read Amc file name
     * \return The Amc file name (path)
     */
    std::string file_name() const;
    /**
     * \brief
     * \return
     */
    double skeleton_scale() const;
    /**
     * \brief
     * \return
     */
    std::string skeleton_file_name() const;
    /**
     * \brief Get the number of frames
     */
    int32_t frame_num() const;
    /**
     * \brief
     */
    int32_t frame_offset() const;
    /**
     * \brief
     * \param[in] frame_num
     * \return
     */
    int32_t posture_idx(const int32_t frame_idx);
    /**
     * \brief
     * \param[in] frame_idx
     * \return Posture of the corresponding frame frame_num
     */
    Posture *posture_ptr(const int32_t frame_idx) const;
    /**
     * \brief
     */
    math::Vector6dColl_t joint_spatial_pos(const int32_t frame_idx) const;
    /**
     * \brief
     */
    math::SpatialTemporalVector6d_t whole_sequence() const;
    /**
     * \brief
     * \param[in] offset
     */
    void set_frame_offset(const int32_t offset);
    /**
     * \brief Set all postures to default posture
     */
    void set_postures_to_default();
    /**
     * \brief
     */
    void set_default_joint_spatial_pos();
    /**
     * \brief Set the target posture to the specified frame
     * \param[in] frame_idx The specified frame
     * \param[in] posture   The target posture
     */
    void set_posture(const int32_t frame_idx, const Posture &posture);
    /**
     * \brief
     */
    void set_joint_spatial_pos(const int32_t frame_idx, const math::Vector6dColl_t &pos6d_coll);
    /**
     * \brief
     * \param[in] frame_idx
     * \param[in] degree_angle
     * \param[in] bone_idx
     */
    void set_bone_rotation(
            const int32_t frame_idx,
            const math::Vector3d_t &degree_angle,
            const int32_t bone_idx
            );
    /**
     * \brief
     * \param[in] frame_idx
     * \param[in] pos
     */
    void set_root_pos(const int32_t frame_idx, const math::Vector3d_t &pos);
    /**
     * \brief
     * \param[in] file_name
     * \param[in] skeleton_scale Adjust the translational parameter.
     * scale should be consistent with the scale used in Skeleton, and the
     * default value is 0.06
     * \retval TRUE  Success in reading the given amc file
     * \retval FALSE Fail in reading the given amc file
     */
    bool ReadAmcFile(const std::string &file_name, const double skeleton_scale);
    /**
     * \brief Output the containing postures as an AMC file
     * \param[in] file_name      The output file name
     * \param[in] skeleton_scale The skeleton scale
     * \return The frame number wrote into the file
     */
    int32_t WriteAmcFile(const std::string &file_name, const double skeleton_scale);
    /**
     * \brief Output the containing spatial-temporal-joint-position as an AMC file
     * \param[in] file_name      The output file name
     * \param[in] skeleton_scale The skeleton scale
     * \param[in] st_joint_pos6d The target joint spaital position
     */
    void OutputAmcFile(
            const std::string &file_name,
            const double skeleton_scale,
            const math::SpatialTemporalVector6d_t &target_st_joint_pos6d
            );
    void OutputAmcFile(
            const std::string &file_name,
            const double skeleton_scale
            );

private:

    int32_t frame_num_; ///< Number of frames in the motion
    int32_t frame_offset_;

    std::string file_name_;
    std::shared_ptr<Skeleton> skeleton_;
    /**
     * Root position and all bone rotation angles for each frame (as read from
     * AMC file)
     */
    std::unique_ptr<PostureColl_t> posture_coll_;
    std::unique_ptr<math::SpatialTemporalVector6d_t> st_joint_pos6d_;
    std::unique_ptr<std::string> amc_header_;
};

} // namespace acclaim {

#endif // #ifndef _ACCLAIM_MOTION_H_
