#ifndef _MATH_LINEAR_SYSTEM_SOLVER_H_
#define _MATH_LINEAR_SYSTEM_SOLVER_H_

#include "math_type.h"

namespace math {

class LinearSystemSolver
{

public:

    virtual ~LinearSystemSolver()
    {}

    virtual std::string id() const = 0;

    virtual math::VectorNd_t Solve(
            const math::MatrixN_t &coef_mat,
            const math::VectorNd_t &desired_vector
            ) const = 0;

protected:

    LinearSystemSolver()
    {}

private:

};

} // namespace math {

#endif // #ifndef _MATH_LINEAR_SYSTEM_SOLVER_H_
