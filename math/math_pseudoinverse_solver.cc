#include "math_pseudoinverse_solver.h"

using namespace Eigen;

namespace math {

// public func.

const char *PseudoinverseSolver::tag()
{
    return "PseudoinverseSolver";
}

PseudoinverseSolver::PseudoinverseSolver()
    :LinearSystemSolver()
{
}

PseudoinverseSolver::~PseudoinverseSolver()
{
}

std::string PseudoinverseSolver::id() const
{
    return std::string(PseudoinverseSolver::tag());
}

math::VectorNd_t PseudoinverseSolver::Solve(
        const math::MatrixN_t &coef_mat,
        const math::VectorNd_t &desired_vector
        ) const
{//TO DO

	VectorNd_t angularVector(coef_mat.cols());

	MatrixN_t JacobianInverse(coef_mat.cols(), coef_mat.rows());

	//JacobianInverse = (coef_mat.transpose()*coef_mat).inverse()*coef_mat.transpose();
	
	JacobiSVD<MatrixN_t> svd(coef_mat, ComputeThinU | ComputeThinV);

	MatrixXd sigma(3,3);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (i == j)
			{
				sigma(i, j) = svd.singularValues()[i];
			}
			else
			{
				sigma(i, j) = 0;
			}
		}
	}

	JacobianInverse = svd.matrixV()*sigma.inverse();
	JacobianInverse = JacobianInverse*svd.matrixU().transpose();
	angularVector = JacobianInverse*desired_vector;

    //return math::VectorNd_t();
	return angularVector;
}

// protected func.

// private func.

} // namespace math {
