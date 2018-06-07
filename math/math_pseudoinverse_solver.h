#ifndef _MATH_PSEUDOINVERSE_SOLVER_H_
#define _MATH_PSEUDOINVERSE_SOLVER_H_

#include "math_linear_system_solver.h"

namespace math {

class PseudoinverseSolver final : public LinearSystemSolver
{

public:

    static const char *tag();

    PseudoinverseSolver();
    PseudoinverseSolver(const PseudoinverseSolver &other) = default;
    virtual ~PseudoinverseSolver();
    PseudoinverseSolver &operator=(const PseudoinverseSolver &other) = default;

    virtual std::string id() const override;
    /**
     * \brief Solve the linear system Ax = b, i.e., b = A^{-1}x
     * \param[in] coef_mat The coefficient matrix A
     * \param[in] desired_vector The desired vector b
     * \return The unknown vector x computed by Pseudoinverse
     */
    virtual math::VectorNd_t Solve(
            const math::MatrixN_t &coef_mat,
            const math::VectorNd_t &desired_vector
            ) const override;

protected:

private:

};

} // namespace math {

#endif // #ifndef _MATH_PSEUDOINVERSE_SOLVER_H_
