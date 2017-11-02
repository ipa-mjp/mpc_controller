
#ifndef MPC_CONTROLLER_MPC_SOLVER_ACADO_MPC_SOLVER_ACADO_H
#define MPC_CONTROLLER_MPC_SOLVER_ACADO_MPC_SOLVER_ACADO_H

#include <mpc_controller/data_types.h>
#include <mpc_controller/inverse_jacobian_calculation/inverse_jacobian_calculation.h>

//ACADO
#include <acado/acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>

#include <kdl/jntarray.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

#include <ros/ros.h>

#include <iostream>
#include <fstream>

namespace nmpc {

	using namespace ACADO;

	class ModelPredictiveControlACADO
	{
		public:

			ModelPredictiveControlACADO() = default;

			bool initializeClassMembers(const JacobianMatrix& jacobian_mat);

			void setJacobianData(const JacobianMatrix& jacobian_data);
			Eigen::MatrixXd solve(void);//(const Cart6dVector& in_cart_velocities, const JointStates& joint_states);

			Eigen::MatrixXd mpc_solve(void);
		private:

			//Matrix6Xd_t 	jacobian_data_	;	//Jacobian matrix
			JacobianMatrix	jacobian_data_	;	//Jacobian matrix
			 JInvBySVD 		jInc_cals_		;	//Computation PsudoInverse of jacobian matrix using SVD decomposition
	};

	#endif //MPC_CONTROLLER_MPC_SOLVER_ACADO_MPC_SOLVER_ACADO_H

}  // namespace nmpc
