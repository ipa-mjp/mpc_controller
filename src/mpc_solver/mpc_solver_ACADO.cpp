
#include <mpc_controller/mpc_solver/mpc_solver_ACADO.h>


using namespace nmpc;


void ModelPredictiveControlACADO::setJacobianData(const Matrix6Xd_t& jacobian_data)
{
	this->jacobian_data_ = jacobian_data;
}

Eigen::MatrixXd ModelPredictiveControlACADO::solve(void)//(const Cart6dVector& in_cart_velocities, const JointStates& joint_states)
{

	const unsigned int m = this->jacobian_data_.rows();
	const unsigned int n = this->jacobian_data_.cols();

    // INTRODUCE THE VARIABLES:
    // ----------------------------
	DifferentialState	x ( "", m, 1 )	;	//differential state, in our case it is differential velocity
	DifferentialState	y				;	//differential state

	Control u ("", n , 1);					//control variable, in our case joint velocity

	DifferentialEquation f;					//differential equation

	x.clearStaticCounters();
	y.clearStaticCounters();
	u.clearStaticCounters();

	const double t_start = 0.0;
	const double t_end 	 = 1.0;
	const int horizon    = 10;

	//CALCULATE JACOBIAN MATRIX AND STORE INTO ACADO VARIABLES/MATRIX
	//---------------------------------------------------------------
	Eigen::MatrixXd jInv = this->jInc_cals_.calculate(this->jacobian_data_);

	DMatrix J_inv;	// Jacobian inverse
	DMatrix J;		// Jacobian

	J_inv = jInv;
	J = this->jacobian_data_;

	//READ A VELOCITY VECTOR
	//-----------------------------------------------------------------
	DVector x0;	DVector in_cart_velocities(6);
	in_cart_velocities.setAll(0.0);
	in_cart_velocities(0) = 0.454075;
	in_cart_velocities(1) = 0.0;
	in_cart_velocities(5) = 0.0;
	x0 = in_cart_velocities;

	DVector xEnd; xEnd = in_cart_velocities * 0;

	//DEFINE A DIFFERENTIAL EQUATION
	//----------------------------------------------------------------
	f << dot(x) == J * u;
	f << dot(y) == 0.5*( (x-x0).transpose() * (x-x0));	// minimize in velocity error

	// DEFINE AN OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
	OCP ocp( t_start, t_end, horizon );

	ocp.minimizeMayerTerm( y );     // running cost
	ocp.subjectTo( f );
	ocp.subjectTo( AT_END  , x == xEnd);
	ocp.subjectTo( AT_START, y == (double)(x0.transpose() * x0));

    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
	OptimizationAlgorithm algorithm(ocp)					;
	algorithm.set( MAX_NUM_ITERATIONS, 20 )					;
	algorithm.set( DISCRETIZATION_TYPE,	MULTIPLE_SHOOTING )	;
	algorithm.set( LEVENBERG_MARQUARDT, 1e-5 )				;

	ros::Time begin = ros::Time::now();

	algorithm.solve();

    ros::Time end = ros::Time::now();
    ros::Duration d = end-begin;
    ROS_WARN_STREAM("Time: " << d.toSec());

    VariablesGrid controls_, diffStates ;
    algorithm.getControls(controls_);
    algorithm.getDifferentialStates(diffStates);

	GnuplotWindow window;
		window.addSubplot(diffStates(0), "DIFFERENTIAL STATE  x");
	    window.addSubplot(diffStates(1),"DIFFERENTIAL STATE  y");
	    window.addSubplot(controls_,"CONTROL  u"   );
	window.plot( );


    std::cout<< controls_.getFirstVector() << std::endl;

  return controls_.getFirstVector();
}


