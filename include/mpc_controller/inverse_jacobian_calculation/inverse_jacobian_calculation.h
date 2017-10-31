
#ifndef MPC_CONTROLLER_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_H
#define MPC_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_H

#include <Eigen/Core>
#include <Eigen/LU>				//inverse of matrix
#include <boost/shared_ptr.hpp>

namespace nmpc
{

	class JInvBySVD
	{
		public:
			JInvBySVD() = default;
			//~JInvBySVD() = delete;

			Eigen::MatrixXd calculate(const Eigen::MatrixXd& jacobian) const;
			void calculate(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& jacobianInv) const;

	};

	class JInvByDirect
	{
			JInvByDirect() = default;
			//~JInvByDirect() = delete;

			Eigen::MatrixXd calculate(const Eigen::MatrixXd& jacobian) const;
			void calculate(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& jacobianInv) const;
	};

	#endif //MPC_CONTROLLER_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_H

}
