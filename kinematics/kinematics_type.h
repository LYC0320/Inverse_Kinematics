#ifndef _KINEMATICS_TYPE_H_
#define _KINEMATICS_TYPE_H_

#include <memory>
#include <vector>
#include <map>
#include <string>
#include "math_fwd.h"
#include "kinematics_fwd.h"

namespace kinematics {

//typedef enum NumericalIk: int32_t
//{
//    Pseudoinverse,      // = 0
//    DampedLeastSquares, // = 1
//    Num,                // = 2
//    Null = Num,
//} NumericalIkMethod_t;

typedef std::vector<ArticIdx> ArticIdxColl_t;
typedef std::vector<Pose> PoseColl_t;
} // namespace kinematics {

#endif // #ifndef _KINEMATICS_TYPE_H_
