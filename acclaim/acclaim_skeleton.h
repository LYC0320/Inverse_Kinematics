/*
	skeleton.h

	Definition of the skeleton. 

    Revision 1 - Steve Lin, Jan. 14, 2002
 	Revision 2 - Alla and Kiran, Jan 18, 2002
*/

#ifndef _ACCLAIM_SKELETON_H_
#define _ACCLAIM_SKELETON_H_

#include <string>
#include "posture.h"
#include "acclaim_bone.h"

namespace acclaim {
/**
 * Use this parameter to adjust the size of the skeleton.
 * The default value is 0.06 that creates a human skeleton of 1.7m in height approximately.;
 */
static const double MoCapScale()
{return double{0.06};}

class Skeleton final
{

public: 
    /**
     * \brief Get the articulated index of skeleton root
     * \return Articulated index of the skeleton root
     */
    static const int32_t root_idx()
    {return 0;}
    /**
     * \brief Constructor
     * \param[in] asf_file_name Asf file name
     * \param[in] scale Adjust the size of the skeleton.
     * The default value is 0.06 (MOCAP_SCALE), and it creates a 1.7m human
     * skeleton approximately
     */
    Skeleton(const std::string &asf_file_name, const double scale);  
    Skeleton(const Skeleton &) = delete;
    /**
     * \brief Destructor
     */
    virtual ~Skeleton();
    Skeleton &operator=(const Skeleton &) = delete;
    /**
     * \brief Get the read Asf file name
     * \return The Asf file name (path)
     */
    std::string file_name() const;
    /**
     * \brief Get the skeleton scale
     * \return The skeleton scale
     */
    double scale() const;
    /**
     * \brief Get total number of bones of this skeleton
     * return Number of bones
     */
    int32_t bone_num() const;
    /**
     * \brief
     */
    math::Vector3d_t root_pos() const;
    /**
     * \brief Get root node's address, for accessing bone data
     * \return Bone structure
     */
    Bone* root_bone();
    /**
     * \brief Get the body length, specified by bone_idx
     */
    double bone_length(const int32_t bone_idx) const;
    /**
     * \brief
     */
    std::string bone_name(const int32_t bone_idx) const;
    /**
     * \brief
     */
    int32_t bone_idx(const std::string &name) const;
    /**
     * \brief
     */
    math::Vector3d_t bone_local_dir(const int32_t bone_idx) const;
    /**
     * \brief Initialize root at (0, 0, 0) and other rotation angles to 0
     */
    //void set_base_posture();
    /**
     * \brief Get the bone instance indicated by the bone_idx
     * \param[in] bone_idx
     * \return
     */
    const Bone *bone_ptr(const int32_t bone_idx) const;

private:

    /**
     * \brief Parse the skeleton (.asf) file
     * \param[in] asf_file_name
     * \param[in] scale
     */
    void ReadAsfFile(const std::string &asf_file_name, const double scale);
    /**
     * \brief
     */
    int32_t ComputeBoneNum(const std::string &asf_file_name);
    //This recursive function traverces skeleton hierarchy 
    //and returns a pointer to the bone with index - bIndex
    //ptr should be a pointer to the root node 
    //when this function first called
    /**
     * \brief
     * \param[in] ptr
     * \param[in] bIndex
     */
    Bone* GetBone(Bone *ptr, const int32_t bIndex);

    //This function sets sibling or child for parent bone
    //If parent bone does not have a child, 
    //then pChild is set as parent's child
    //else pChild is set as a sibling of parents already existing child
    /**
     * \brief Set sibling or child for parent bone
     */
    int32_t SetChildrenAndSibling(const int32_t parent, Bone *pChild);
    /**
     * \brief Rotate all bones' direction vector (dir) from global to local
     * coordinate system
     */
    void RotateBoneDirToLocalCoordSystem();
    void ComputeRotationToParentCoordSystem(Bone *bone);
    void ComputeRotationParentChild(Bone *parent, Bone *child);
    void SetBoneShape(Bone *bone);

    math::Vector3d_t root_pos_; ///< Root position in world coordinate system
    Bone *root_bone_;							///< Pointer to the root bone, m_RootBone = &bone[0]
    Bone *bone_list_;
    std::string file_name_;
    double scale_;
    int32_t bone_num_;
    int32_t bone_count_;
};

int32_t numBonesInSkel(Bone &item);
int32_t movBonesInSkel(Bone &item);

} // namespace acclaim {

#endif // #ifndef _ACCLAIM_SKELETON_H_
