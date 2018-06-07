#ifndef _POSTURE_H_
#define _POSTURE_H_

#include <memory>
#include <vector>
#include "acclaim_fwd.h"
#include "acclaim_body_idx.h"
#include "math_type.h"

typedef std::vector<Posture> PostureColl_t;
/**
 * Root position and all bone, including root, rotatin angles
 */
class Posture
{

public:

    static Posture ToRadianBoneRotation(const Posture &degree_posture);
    static Posture ToDegreeBoneRotation(const Posture &radian_posture);
    /**
     * \brief
     */
    Posture();
    /**
     * \brief
     * \param[in] size
     */
    explicit Posture(const int32_t size);
    /**
     * \brief
     * \param[in] other
     */
    Posture(const Posture &other);
    /**
     * \brief
     */
    virtual ~Posture();
    /**
     * \brief
     * \param[in] other
     */
    Posture &operator=(const Posture &other);
    /**
     * \brief
     * \return
     */
    int32_t size() const;
    /**
     * \brief
     * \return
     */
    math::Vector3d_t root_pos() const;
    /**
     * \brief
     * \return
     */
    math::Vector3dColl_t bone_rotations() const;
    /**
     * \brief
     * \param[in] bone_idx
     * \return
     */
    math::Vector3d_t bone_rotation(const int32_t bone_idx) const;
    /**
     * \brief
     * \return
     */
    math::Vector3dColl_t bone_translations() const;
    /**
     * \brief
     * \param[in] bone_idx
     * \return
     */
    math::Vector3d_t bone_translation(const int32_t bone_idx) const;
    /**
     * \brief
     * \param[in]
     */
    void set_root_pos(const math::Vector3d_t &pos);
    /**
     * \brief
     * \param[in]
     */
    void set_bone_rotations(const math::Vector3dColl_t &angle3d_coll);
    /**
     * \brief
     * \param[in]
     */
    void set_bone_rotation(const int32_t bone_idx, const math::Vector3d_t &angle3d);
    /**
     * \brief
     * \param[in]
     */
    void set_bone_translations(const math::Vector3dColl_t &xlt_coll);
    /**
     * \brief
     * \param[in]
     */
    void set_bone_translation(const int32_t bone_idx, const math::Vector3d_t &xlt);
    /**
     * \brief
     * \param[in]
     */
    void reset(const int32_t size);
    /**
     * \brief
     * \return
     */
    bool is_empty() const;
    /**
     * \brief
     * \param[in] a_kPoseture
     * \param[in] epsilon
     */
    bool is_approx(
            const Posture &other,
            const double epsilon
            ) const;

protected:

private:

    /**
     * Rotation(x, y, z) of all bones at a particular time frame in their local coordinate system.
     * If a particular bone does not have a certain Dof,
     * the corresponding rotation is 0.
     * The order of the bones in the container corresponding to their IDs in ASF file: root,
     * lhipjoint, lfemur....
     */
    int32_t size_;
    std::unique_ptr<math::Vector3d_t> root_pos_;
    std::unique_ptr<math::Vector3dColl_t> bone_rotations_;
    std::unique_ptr<math::Vector3dColl_t> bone_translations_;
};

#endif // #ifndef _POSTURE_H_
