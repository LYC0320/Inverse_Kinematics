#include "gl_utils.h"
#include <algorithm>

namespace glmat4 {

void SetIdentity(double mat4[glmat4::Size])
{
    std::fill(mat4, mat4 + glmat4::Size, double{0.0});
    mat4[0] = double{1.0};
    mat4[5] = double{1.0};
    mat4[10] = double{1.0};
    mat4[15] = double{1.0};
}

} // namespace glmat4 {
