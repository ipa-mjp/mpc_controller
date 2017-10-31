
#include <Eigen/Core>
#include <Eigen/SVD>
#include <mpc_controller/inverse_jacobian_calculation/inverse_jacobian_calculation.h>

using namespace nmpc;

Eigen::MatrixXd JInvBySVD::calculate(const Eigen::MatrixXd& jacobian) const
{

	//Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);

	//singular values lie on diagonal of matrix, easily invert
	Eigen::VectorXd singularValues = svd.singularValues();
	Eigen::VectorXd singularValuesInv = Eigen::VectorXd::Zero(singularValues.rows());

	for (uint32_t i = 0; i < singularValues.rows(); ++i)
	{
		double denominator = singularValues(i) * singularValues(i);
		//singularValuesInv(i) = 1.0 / singularValues(i);
		singularValuesInv(i) = (singularValues(i) < 1e-6) ? 0.0 : singularValues(i) / denominator;
	}

	Eigen::MatrixXd result = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();

	return result;
}

void JInvBySVD::calculate(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& jacobianInv) const
{
	jacobianInv = this->calculate(jacobian);
}

Eigen::MatrixXd JInvByDirect::calculate(const Eigen::MatrixXd& jacobian) const
{
    Eigen::MatrixXd result;
    Eigen::MatrixXd jac_t = jacobian.transpose();
    uint32_t rows = jacobian.rows();
    uint32_t cols = jacobian.cols();

    if (cols >= rows)
    {
        result = jac_t * (jacobian * jac_t).inverse();
    }
    else
    {
        result = (jac_t * jacobian).inverse() * jac_t;
    }

    return result;
}

void JInvByDirect::calculate(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& jacobianInv) const
{
	jacobianInv = this->calculate(jacobian);
}

