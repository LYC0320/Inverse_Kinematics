#ifndef _MATH_DAMPED_LEAST_SQUARES_SOLVER_H_
#define _MATH_DAMPED_LEAST_SQUARES_SOLVER_H_

#include "math_linear_system_solver.h"

namespace math {

class DampedLeastSquaresSolver final : public LinearSystemSolver
{

public:

    static const char *tag();

    DampedLeastSquaresSolver();
    explicit DampedLeastSquaresSolver(const double damping_constant);
    DampedLeastSquaresSolver(const DampedLeastSquaresSolver &other) = default;
    virtual ~DampedLeastSquaresSolver();
    DampedLeastSquaresSolver &operator=(const DampedLeastSquaresSolver &other) = default;

    virtual std::string id () const override;
    double damping_constant() const;
    /**
     * \brief Set the damping constant lambda
     * \param[in] damping_constant The damping constant lambda
     */
    void set_damping_constant(const double damping_constant);
    /**
     * \brief Solve the linear system Ax = b, i.e., b = A^{-1}x
     * \param[in] coef_mat The coefficient matrix A
     * \param[in] desired_vector The desired vector b
     * \return The unknown vector x computed by A^{T}(A * A^{T} + lambda^2 * I)^{-1} * b
     */
    virtual math::VectorNd_t Solve(
            const math::MatrixN_t &coef_mat,
            const math::VectorNd_t &desired_vector
            ) const override;

protected:

private:

    double damping_constant_;
};

} // namespace math {

#endif // #ifndef _MATH_DAMPED_LEAST_SQUARES_SOLVER_H_
