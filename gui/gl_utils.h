#ifndef _GL_UTILS_H_
#define _GL_UTILS_H_

#include "global_def.h"

namespace glmat4 {
/**
 * \brief Get the array size (element number) of a homogeneous matrix
 * \return The array size of a homogeneous matrix
 */
//static const int32_t Size()
//{return 16;}
enum { Size = 16 };
    
/**
 * \brief Get the row number of a homogeneous matrix
 * \return The row number of a homogeneous matrix
 */
static const int32_t Rows()
{return 4;}
/**
 * \brief Get the column number of a homogeneous matrix
 * \return The column number of a homogeneous matrix
 */
static const int32_t Cols()
{return 4;}

namespace idx {

/**
 * \brief The column-major array index of a homogeneous matrix
 *  /            \
 *  | 0  4  8  12 |
 *  | 1  5  9  13 |
 *  | 2  6 10  14 |
 *  | 3  7 11  15 |
 *  \            /
 */
static const int32_t XltX()
{return 12;}
static const int32_t XltY()
{return 13;}
static const int32_t XltZ()
{return 14;}

} // namespace glmat4::idx {

/**
 * \brief Set the identity homogeneous matrix
 *  /            \
 *  | 1  0  0  0 |
 *  | 0  1  0  0 |
 *  | 0  0  1  0 |
 *  | 0  0  0  1 |
 *  \            /
 * \param[in]  mat4 The given 4x4 matrix
 * \param[out] mat4 The output identity homogeneous matrix
 */
void SetIdentity(double mat4[glmat4::Size]);

} // namespace glmat4 {

#endif // #ifndef _GL_UTILS_H_
