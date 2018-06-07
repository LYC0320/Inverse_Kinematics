#ifndef _ACCLAIM_BONE_H_
#define _ACCLAIM_BONE_H_

#include <string>
#include "math_type.h"

namespace acclaim {

// Bone segment names used in ASF file
// this structure defines the property of each bone segment, including its connection to other bones,
// DOF (degrees of freedom), relative orientation and distance to the outboard bone 
struct Bone
{
    struct Bone *sibling;		///< Pointer to the sibling (branch bone) in the hierarchy tree 
    struct Bone *child;			///< Pointer to the child (outboard bone) in the hierarchy tree 
    struct Bone *parent;        ///< Pointer to the parent (inboard bone) in the hierarchy tree

    int idx;					///< Bone index

    math::Vector3d_t dir;				///< Unit vector describes the direction from local origin to 
    // the origin of the child bone 
    // Notice: stored in local coordinate system of the bone

    double length;				///< Bone length  

    math::Vector3d_t axis;            ///< Orientation of each bone's local coordinate
                                //system as specified in ASF file (axis field)

    double aspx, aspy;			///< aspect ratio of bone shape

    int dof;					///< number of bone's degrees of freedom 
    int dofx, dofy, dofz;		///< degree of freedom mask in x, y, z axis (local)
    int doftx, dofty, doftz;

    std::string name;
    // rotation matrix from the local coordinate of this bone to the local coordinate system of it's parent
    double rot_parent_current[4][4];			

    //Rotation angles for this bone at a particular time frame (as read from AMC file) in local coordinate system, 
    //they are set in the SetPosture function before dispay function is called
    //math::Vector3d_t dr;
};

} // namespace acclaim {

#endif // #ifndef _ACCLAIM_BONE_H_
