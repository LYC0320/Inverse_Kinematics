#include "math_damped_least_squares_solver.h"

namespace math {

// public func.

const char *DampedLeastSquaresSolver::tag()
{
    return "DampedLeastSquaresSolver";
}

DampedLeastSquaresSolver::DampedLeastSquaresSolver()
    :LinearSystemSolver(),
    damping_constant_(double{0.0})
{
}

DampedLeastSquaresSolver::DampedLeastSquaresSolver(const double damping_constant)
    :LinearSystemSolver(),
    damping_constant_(damping_constant)
{
}

DampedLeastSquaresSolver::~DampedLeastSquaresSolver()
{
}

std::string DampedLeastSquaresSolver::id() const
{
    return std::string(DampedLeastSquaresSolver::tag());
}

double DampedLeastSquaresSolver::damping_constant() const
{
    return damping_constant_;
}

void DampedLeastSquaresSolver::set_damping_constant(const double damping_constant)
{
    damping_constant_ = damping_constant;
}

math::VectorNd_t DampedLeastSquaresSolver::Solve(
        const math::MatrixN_t &coef_mat,
        const math::VectorNd_t &desired_vector
        ) const
{	
    return math::VectorNd_t();
}

// protected func.

// private func.

} // namespace math {
