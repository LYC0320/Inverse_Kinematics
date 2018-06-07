#include "acclaim_skeleton.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include "console_log.h"
#include "math_utils.h"

namespace acclaim {

Skeleton::Skeleton(const std::string &asf_file_name, const double scale)
    :root_pos_(math::Vector3d_t::Zero()),
    root_bone_(nullptr),
    bone_list_(nullptr),
    file_name_(asf_file_name),
    scale_(scale),
    bone_num_(0),
    bone_count_(0)
{
    bone_num_ = this->ComputeBoneNum(asf_file_name);
    bone_list_ = new Bone[bone_num_];

    bone_list_[0].name = std::string("root");
    bone_count_ = 1;
    //Initializaton
    bone_list_[Skeleton::root_idx()].idx = root_idx();   // root of hierarchy
    root_bone_ = &bone_list_[Skeleton::root_idx()];
    bone_list_[Skeleton::root_idx()].sibling = nullptr;
    bone_list_[Skeleton::root_idx()].child = nullptr;
    bone_list_[Skeleton::root_idx()].parent = nullptr;
    bone_list_[Skeleton::root_idx()].dir[0] = 0.;
    bone_list_[Skeleton::root_idx()].dir[1] = 0.;
    bone_list_[Skeleton::root_idx()].dir[2] = 0.;
    bone_list_[Skeleton::root_idx()].axis.x() = 0.;
    bone_list_[Skeleton::root_idx()].axis.y() = 0.;
    bone_list_[Skeleton::root_idx()].axis.z() = 0.;
    bone_list_[Skeleton::root_idx()].length = 0.05;
    bone_list_[Skeleton::root_idx()].dof = 6;
    bone_list_[Skeleton::root_idx()].dofx = 1;
    bone_list_[Skeleton::root_idx()].dofy = 1;
    bone_list_[Skeleton::root_idx()].dofz = 1;
    bone_list_[Skeleton::root_idx()].doftx = 1;
    bone_list_[Skeleton::root_idx()].dofty = 1;
    bone_list_[Skeleton::root_idx()].doftz = 1;

    root_pos_ = math::Vector3d_t::Zero();
    // build hierarchy and read in each bone's DOF information
    this->ReadAsfFile(asf_file_name, scale_);

    //transform the direction vector for each bone from the world coordinate system
    //to it's local coordinate system
    this->RotateBoneDirToLocalCoordSystem();

    //Calculate rotation from each bone local coordinate system to the coordinate system of its parent
    //store it in rot_parent_current variable for each bone
    this->ComputeRotationToParentCoordSystem(root_bone_);

    //Set the aspect ratio of each bone
    this->SetBoneShape(root_bone_);
}

Skeleton::~Skeleton()
{
    delete [] bone_list_;
    bone_list_ = nullptr;
}

std::string Skeleton::file_name() const
{
    return file_name_;
}

double Skeleton::scale() const
{
    return scale_;
}

int32_t Skeleton::bone_num() const
{
    return bone_num_;
}

math::Vector3d_t Skeleton::root_pos() const
{
    return root_pos_;
}

Bone *Skeleton::root_bone()
{
    return root_bone_;
}


double Skeleton::bone_length(const int32_t bone_idx) const
{
    return bone_list_[bone_idx].length;
}

std::string Skeleton::bone_name(const int32_t bone_idx) const
{
    int32_t i = 0;
    while (bone_list_[i].idx != bone_idx && i++ < this->bone_num())
        ;

    return bone_list_[i].name;
}

int32_t Skeleton::bone_idx(const std::string &name) const
{
    for (int32_t bone_idx = 0; bone_idx < bone_num_; ++bone_idx)
    {
        if (name == bone_list_[bone_idx].name)
        {
            return bone_idx;
        }
    }

    for (int32_t idx = 0; idx < bone_num_; ++idx)
    {
        std::cout << "idx: " << idx << ", " << bone_list_[idx].name << std::endl;
    }
    
    std::cout << "bone num: " << bone_num_ << std::endl;
    LOGERR << "invalid bone name: " << name << std::endl;
    assert(FALSE);
    return -1;
}


math::Vector3d_t Skeleton::bone_local_dir(const int32_t bone_idx) const
{
    return bone_list_[bone_idx].dir;
}

//void Skeleton::set_base_posture()
//{
//    root_pos_ = math::Vector3d_t::Zero();
//    for (int32_t i = 0; i < this->bone_num(); ++i)
//    {
//        bone_list_[i].dr = math::Vector3d_t::Zero();
//    }
//}

const Bone *Skeleton::bone_ptr(const int32_t bone_idx) const
{
    return &bone_list_[bone_idx];
}

// private func.

void Skeleton::ReadAsfFile(const std::string &asf_file_name, const double scale)
{
    std::ifstream input_stream(asf_file_name, std::ios::in);
    if (input_stream.fail())
    {
        return ;
    }
    else
    {
        input_stream.close();
        input_stream.open(asf_file_name, std::ios::out);
    }

    // ignore header information
    char str[2048], keyword[256];
    while (1)
    {
        input_stream.getline(str, 2048);
        sscanf(str, "%s", keyword);
        if (strncmp(keyword, ":bonedata", strlen(":bonedata")) == 0)
        {
            break;
        }
    }

    // Read bone information: global orientation and translation, DOF.
    input_stream.getline(str, 2048);
    char part[256], *token;
    double length = 0.0;

    bool done = false;
    for(int32_t i = 1; !done && i < this->bone_num(); i++)
    {
        bone_list_[i].dof=0;
        bone_list_[i].dofx = bone_list_[i].dofy = bone_list_[i].dofz = 0;
        bone_list_[i].doftx = bone_list_[i].dofty = bone_list_[i].doftz = 0;
        ++bone_count_;

        while(1)
        {
            input_stream.getline(str, 2048);
            sscanf(str, "%s", keyword);

            if (strncmp(keyword, "end", strlen("end")) == 0)
            {
                break;
            }

            if (strncmp(keyword, ":hierarchy", strlen(":hierarchy")) == 0)
            {
                bone_count_ -= 1;
                done=true;
                break;
            }

            //id of bone
            if (strncmp(keyword, "id", strlen("id")) == 0)
            {
                bone_list_[i].idx = bone_count_ - 1;
            }
            //name of the bone
            if (strncmp(keyword, "name", strlen("name")) == 0)
            {
                char name_buf[256] = {'\0'};
                sscanf(str, "%s %s", keyword, part);
                sscanf(str, "%s %s", keyword, name_buf);
                bone_list_[i].name = std::string(name_buf);
            }

            //this line describes the bone's direction vector in global coordinate
            //it will later be converted to local coorinate system
            if (strncmp(keyword, "direction", strlen("direction")) == 0)
            {
                sscanf(str, "%s %lf %lf %lf", keyword, &bone_list_[i].dir[0], &bone_list_[i].dir[1], &bone_list_[i].dir[2]);
            }

            //length of the bone
            if (strncmp(keyword, "length", strlen("length")) == 0)
            {
                sscanf(str, "%s %lf", keyword, &length);
            }

            //this line describes the orientation of bone's local coordinate
            //system relative to the world coordinate system
            if (strncmp(keyword, "axis", strlen("axis")) == 0)
            {
                sscanf(str, "%s %lf %lf %lf", keyword, &bone_list_[i].axis[0], &bone_list_[i].axis[1], &bone_list_[i].axis[2]);
            }

            // this line describes the bone's dof
            if (strncmp(keyword, "dof", strlen("dof")) == 0)
            {
                token=strtok(str, " ");
                bone_list_[i].dof=0;
                while (token != nullptr)
                {
                    if(strncmp(token, "rx", strlen("rx")) == 0)
                    {
                        bone_list_[i].dofx = 1;
                        bone_list_[i].dof++;
                    }
                    else if(strncmp(token, "ry", strlen("ry")) == 0)
                    {
                        bone_list_[i].dofy = 1;
                        bone_list_[i].dof++;
                    }
                    else if(strncmp(token, "rz", strlen("rz")) == 0)
                    {
                        bone_list_[i].dofz = 1;
                        bone_list_[i].dof++;
                    }
                    else if(strncmp(token, "tx", strlen("tx")) == 0)
                    {
                        bone_list_[i].doftx = 1;
                        bone_list_[i].dof++;
                    }
                    else if(strncmp(token, "ty", strlen("ty")) == 0)
                    {
                        bone_list_[i].dofty = 1;
                        bone_list_[i].dof++;
                    }
                    else if(strncmp(token, "tz", strlen("tz")) == 0)
                    {
                        bone_list_[i].doftz = 1;
                        bone_list_[i].dof++;
                    }
                    else if(strncmp(token, "dof", strlen("dof")) == 0)
                    {
                    }
                    else
                    {
                        printf("UNKNOWN %s\n",token);
                    }

                    //bone_list_[i].dof++;
                    token=strtok(nullptr, " ");
                }
            }
        }
        bone_list_[i].length = length * scale;
        //init child/sibling to nullptr, it will be assigned next (when hierarchy read)
        bone_list_[i].sibling = nullptr;
        bone_list_[i].child = nullptr;
        bone_list_[i].parent = nullptr;
    }

    // read and build the hierarchy of the skeleton
    char *part_name;
    int32_t j, parent;

    //skip "begin" line
    input_stream.getline(str, 2048);

    //Assign parent/child relationship to the bones
    while(1)
    {
        //read next line
        input_stream.getline(str, 2048);
        sscanf(str, "%s", keyword);

        //check if we are done
        if(strncmp(keyword, "end", strlen("end")) == 0)
        {
            break;
        }
        else if (strncmp(keyword, "begin", strlen("begin")) == 0)
        {
            //break;
        }
        else
        {
            //parse this line, it contains parent followed by children
            part_name = strtok(str, " \r\n");
            j=0;
            while (nullptr != part_name)
            {
                if (j==0)
                {
                    parent = this->bone_idx(part_name);
                }
                else
                {
                    this->SetChildrenAndSibling(parent, &bone_list_[this->bone_idx(part_name)]);
                }
                part_name = strtok(nullptr, " \r\n");
                j++;
            }
        }
    }

    input_stream.close();
}

int32_t Skeleton::ComputeBoneNum(const std::string &asf_file_name)
{
    std::ifstream is(asf_file_name, std::ios::in);
    if (is.fail())
    {
        return 0;
    }

    // ignore header information
    char str[2048], keyword[256];
    do{
        is.getline(str, 2048);	
        sscanf(str, "%s", keyword);
    } while(strncmp(keyword, ":bonedata", strlen(":bonedata")) != 0);


    // count the number of bones
    int numBones = 1; // for 'root' bone  
    do{
        is.getline(str, 2048);	sscanf(str, "%s", keyword);
        if(strncmp(keyword, "name", strlen("name")) == 0) numBones++;
    }while(strncmp(keyword, ":hierarchy", strlen(":hierarchy")) != 0);

    is.close();
    return numBones;
}

/*
   This recursive function traverces skeleton hierarchy
   and returns a pointer to the bone with index - bIndex
   ptr should be a pointer to the root node
   when this function first called
   */
Bone* Skeleton::GetBone(Bone *ptr, const int32_t bIndex)
{
    static Bone *theptr;
    if (nullptr == ptr)
    {
        return nullptr;
    }
    else if (ptr->idx == bIndex)
    {
        theptr = ptr;
        return theptr;
    }
    else
    {
        this->GetBone(ptr->child, bIndex);
        this->GetBone(ptr->sibling, bIndex);
        return theptr;
    }
}

/*
   This function sets sibling or child for parent bone
   If parent bone does not have a child,
   then pChild is set as parent's child
   else pChild is set as a sibling of parents already existing child
   */
int32_t Skeleton::SetChildrenAndSibling(const int32_t parent, Bone *pChild)
{
    Bone *pParent = nullptr;

    //Get pointer to root bone
    pParent = this->GetBone(root_bone_, parent);

    if (nullptr == pParent)
    {
        printf("inbord bone is undefined\n");
        return 0;
    }
    else
    {
        pChild->parent = pParent;
        //if pParent bone does not have a child
        //set pChild as parent bone child
        if (nullptr == pParent->child)
        {
            pParent->child = pChild;
        }
        else
        {
            //if pParent bone already has a child
            //set pChils as pParent bone's child sibling
            pParent=pParent->child;
            while (pParent->sibling != nullptr)
            {
                pParent = pParent->sibling;
            }

            pParent->sibling = pChild;
        }
        return(1);
    }
}

/*
   Transform the direction vector (dir),
   which is defined in character's global coordinate system in the ASF file,
   to local coordinate
   */
void Skeleton::RotateBoneDirToLocalCoordSystem()
{
    for (int32_t i = 1; i < this->bone_num(); i++)
    {
        //Transform dir vector into local coordinate system
        math::Vector3d_t bone_global_dir = bone_list_[i].dir;
        math::RotMat3d_t globalToLocal_mat = math::ComputeRotMatZyx(
                math::ToRadian(-bone_list_[i].axis.x()),
                math::ToRadian(-bone_list_[i].axis.y()),
                math::ToRadian(-bone_list_[i].axis.z())
                );
        math::Vector3d_t bone_local_dir = globalToLocal_mat * bone_global_dir;
        bone_list_[i].dir = bone_local_dir;
	}
}

void Skeleton::SetBoneShape(Bone *bone)
{
    bone[Skeleton::root_idx()].aspx=1;
    bone[Skeleton::root_idx()].aspy=1;
    for (int32_t j = 1; j < this->bone_num(); j++)
    {
        bone[j].aspx = 0.25;
        bone[j].aspy = 0.25;
    }
}

void Skeleton::ComputeRotationParentChild(Bone *parent, Bone *child)
{
    math::RotMat3d_t parentCoord_trans_mat, childCoord_mat, childToParent_mat, childToParent_trans_mat;

    if(child != nullptr)
    {
        parentCoord_trans_mat = math::ComputeRotMatZyx(
                math::ToRadian(-parent->axis.x()),
                math::ToRadian(-parent->axis.y()),
                math::ToRadian(-parent->axis.z())
                );
        childCoord_mat = math::ComputeRotMatXyz(
                math::ToRadian(child->axis.x()),
                math::ToRadian(child->axis.y()),
                math::ToRadian(child->axis.z())
                );
        childToParent_mat = parentCoord_trans_mat * childCoord_mat;
        childToParent_trans_mat = childToParent_mat.transpose();
        math::ToMatrix4(childToParent_trans_mat, child->rot_parent_current);
    }
}

void Skeleton::ComputeRotationToParentCoordSystem(Bone *bone)
{
    math::RotMat3d_t rootCoord_mat, rootCoord_trans_mat;

    rootCoord_mat = math::ComputeRotMatXyz(
            math::ToRadian(bone[Skeleton::root_idx()].axis.x()),
            math::ToRadian(bone[Skeleton::root_idx()].axis.y()),
            math::ToRadian(bone[Skeleton::root_idx()].axis.z())
            );
    rootCoord_trans_mat = rootCoord_mat.transpose();
    math::ToMatrix4(
            rootCoord_trans_mat,
            bone[Skeleton::root_idx()].rot_parent_current
            );

    //Compute rot_parent_current for all other bones
    for (int32_t i = 0; i < this->bone_num(); i++)
    {
        if (bone[i].child != nullptr)
        {
            this->ComputeRotationParentChild(&bone[i], bone[i].child);

            // compute parent child siblings...
            Bone *tmp = nullptr;
            if (bone[i].child != nullptr)
            {
                tmp = (bone[i].child)->sibling;
            }
            while (tmp != nullptr)
            {
                this->ComputeRotationParentChild(&bone[i], tmp);
                tmp = tmp->sibling;
            }
        }
    }
}

int32_t numBonesInSkel(Bone &item)
{
    Bone *tmp = item.sibling;
    int32_t i = 0;
    while (tmp != nullptr)
    {
        if (tmp->child != nullptr)
        {
            i+= numBonesInSkel(*(tmp->child));
        }
        i++;
        tmp = tmp->sibling;
    }
    if (item.child != nullptr)
    {
        return i+1+numBonesInSkel(*item.child);
    }
    else
    {
        return i+1;
    }
}

int32_t movBonesInSkel(Bone &item)
{
    Bone *tmp = item.sibling;
    int32_t i = 0;

    if (item.dof > 0)
    {
        i++;
    }

    while (tmp != nullptr)
    {
        if (tmp->child != nullptr)
        {
            i+= movBonesInSkel(*(tmp->child));
        }

        if (tmp->dof > 0)
        {
            i++;
        }

        tmp = tmp->sibling;
    }

    if (item.child != nullptr)
    {
        return i+movBonesInSkel(*item.child);
    }
    else
    {
        return i;
    }
}

} // namespace acclaim {
