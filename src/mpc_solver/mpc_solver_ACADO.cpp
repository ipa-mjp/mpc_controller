
#include <mpc_controller/mpc_solver/mpc_solver_ACADO.h>


using namespace nmpc;


bool ModelPredictiveControlACADO::initializeClassMembers(const JacobianMatrix& jacobian_mat)
{
	if (jacobian_mat.isZero())	//if jacobian data not filled
	{
		return false;
	}

	jacobian_data_ = jacobian_mat;
	return true;
}


void ModelPredictiveControlACADO::setJacobianData(const JacobianMatrix& jacobian_data)
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
	DifferentialState	y				;	//differential state, NONLINEAR-LEAST SEQUARE PROBLEM

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
	//Eigen::MatrixXd jInv = this->jInc_cals_.calculate(this->jacobian_data_);

	DMatrix J_inv;	// Jacobian inverse
	DMatrix J;		// Jacobian

	//J_inv = jInv;
	J = this->jacobian_data_;

	//READ A VELOCITY VECTOR
	//-----------------------------------------------------------------
	DVector x0;	DVector in_cart_velocities(6);
	in_cart_velocities.setAll(0.0);
	in_cart_velocities(0) = 1.00;//0.454075;
	//in_cart_velocities(1) = 0.2905;
	in_cart_velocities(5) = 0.0;
	x0 = in_cart_velocities;

	DVector xEnd; xEnd = in_cart_velocities * 0;

	//DEFINE A DIFFERENTIAL EQUATION
	//----------------------------------------------------------------
	f << dot(x) == J * u;
	f << dot(y) == 0.5*( (x-x0).transpose() * (x-x0));	// minimize in velocity error, NONLINEAR LEAST SQUARE PROBLEM

	// DEFINE AN OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
	OCP ocp( t_start, t_end, horizon );

	ocp.minimizeMayerTerm( y );     // running cost
	ocp.subjectTo( f );
	//ocp.subjectTo( AT_START  , x == x0);
	ocp.subjectTo( AT_END  , x == xEnd);
	ocp.subjectTo( AT_START, y == (double)(x0.transpose() * x0));
	ocp.subjectTo( -2 <= u <= 5);

    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
	OptimizationAlgorithm algorithm(ocp)					;
	algorithm.set( MAX_NUM_ITERATIONS, 20 )					;
	algorithm.set( DISCRETIZATION_TYPE,	MULTIPLE_SHOOTING );//SINGLE_SHOOTING )	;
	algorithm.set( LEVENBERG_MARQUARDT, 1e-5 )				;
	//algorithm.set( INTEGRATOR_TYPE, INT_RK45 )				;

	ros::Time begin = ros::Time::now();
/*
	GnuplotWindow window1;
		//window1.addSubplot(x, "DIFFERENTIAL STATE  x");
	    //window1.addSubplot(y,"DIFFERENTIAL STATE  y");
	    window1.addSubplot(u,"CONTROL  u"   );

	 GnuplotWindow window2(PLOT_AT_EACH_ITERATION);
		window2.addSubplot(x, "DIFFERENTIAL STATE  x");
		//window1.addSubplot(y,"DIFFERENTIAL STATE  y");
		//window1.addSubplot(u,"CONTROL  u"   );


	algorithm << window1;
//	algorithm << window2;


    LogRecord logRecord( LOG_AT_EACH_ITERATION );
    logRecord.addItem( LOG_DIFFERENTIAL_STATES, "STATES" );
    logRecord.addItem( LOG_CONTROLS, "CONTROLS" );

    algorithm << logRecord;
*/
	algorithm.solve();

    ros::Time end = ros::Time::now();
    ros::Duration d = end-begin;
    ROS_WARN_STREAM("Time: " << d.toSec());

    VariablesGrid controls_, diffStates; //, diffStates1, logcontrols_;
    algorithm.getControls(controls_);
    algorithm.getDifferentialStates(diffStates);


	GnuplotWindow window;
		window.addSubplot(diffStates(0), "DIFFERENTIAL STATE  x");
	    window.addSubplot(diffStates(1),"DIFFERENTIAL STATE  y");
	    window.addSubplot(controls_,"CONTROL  u"   );
	window.plot( );
/*
	GnuplotWindow window5;
	algorithm.getLogRecord( logRecord );
		logRecord.getAll(LOG_DIFFERENTIAL_STATES, diffStates1);
		logRecord.getAll(LOG_CONTROLS, logcontrols_);
		//logRecord.print();

	std::ofstream myStatefile, myConrolFile;
	myStatefile.open("/home/bfb-ws/mpc_ws/src/mpc_controller/results/states.txt");
	for (uint16_t i = 0; i < diffStates1.getNumRows(); ++i)
	{
		myStatefile << diffStates1.getMatrix(i) << std::endl;
		//std::cout << diffStates1.getMatrix(i) << std::endl;
	}
	myStatefile.close();
	myStatefile.open("/home/bfb-ws/mpc_ws/src/mpc_controller/results/control.txt");
	for (uint16_t i = 0; i < logcontrols_.getNumRows(); ++i)
	{
		myConrolFile << logcontrols_.getMatrix(i) << std::endl;
		//std::cout << diffStates1.getMatrix(i) << std::endl;
	}
	myConrolFile.close();
    std::cout<< controls_.getMatrix(0.5) << std::endl;
*/

  return controls_.getFirstVector();
}

/*
Eigen::MatrixXd ModelPredictiveControlACADO::mpc_solve(void)//(const Cart6dVector& in_cart_velocities, const JointStates& joint_states)
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
    DMatrix W(n, n);
    W.setIdentity();
    Function h;

    h << x << u;


	f << dot(x) == J * u;
	f << dot(y) == 0.5*( (x-x0).transpose() * (x-x0));	// minimize in velocity error

	// SETTING UP THE (SIMULATED) PROCESS:
	// -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f,identity );
	Process process;
	process.setDynamicSystem( dynamicSystem,INT_RK45 );
	process.set( ABSOLUTE_TOLERANCE,1.0e-8 );

	 DVector xm( 3 );
	 xm.setZero( );
	 xm( 0 ) = 0.01;

	 std::cout<<"\033[94m"<<"Hello " <<"\033[0m"<<std::endl;

	 process.initializeStartValues( xm );

	 std::cout<<"\033[94m"<<"Hello1 " <<"\033[0m"<<std::endl;
	 // SIMULATE AND GET THE RESULTS:
	     // -----------------------------
	     VariablesGrid up( 1,0.0,1.0,n );

	     up( 0,0 ) = 10.0;
	     up( 1,0 ) = 0.0;

	     DVector p( n ); p.setAll(0.0);
	     p(0) = 350.0;
	     std::cout<<"\033[94m"<<"Hello2 " <<"\033[0m"<<std::endl;
	     process.init( 0.0 );
	     process.run( up,p );

	     std::cout<<"\033[94m"<<"Hello3 " <<"\033[0m"<<std::endl;

/*
	// DEFINE AN OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
	const double t_start = 0.0;
	const double t_end 	 = 1.0;
	const int horizon    = 10;

	OCP ocp( t_start, t_end, horizon );

    // LSQ coefficient matrix
    DMatrix Q(5,5);
    Q(0,0) = 10.0;
    Q(1,1) = 10.0;
    Q(2,2) = 1.0;
    Q(3,3) = 1.0;
    Q(4,4) = 1.0e-8;

    // Reference
    DVector r(5);
    r.setAll( 0.0 );
	/////
    // LSQ coefficient matrix
    DMatrix Q(2,2);
    Q.setIdentity();
    Q = Q*1000;

    // Reference
    DVector r(2);
    r.setAll( 0.0 );
	//////
	//ocp.minimizeMayerTerm( y );     // running cost
    ocp.minimizeLSQ(Q,h,r);
    ocp.subjectTo( f );
    ocp.subjectTo( AT_START  , x == -x0);
	ocp.subjectTo( AT_END  , x == xEnd);
	ocp.subjectTo( AT_START, y == (double)(x0.transpose() * x0));

	 // SETTING UP THE MPC CONTROLLER:
	 // ------------------------------
	RealTimeAlgorithm algorithm(ocp, 0.05)					;
	algorithm.set( INTEGRATOR_TYPE, INT_RK78 )				;
	algorithm.set( MAX_NUM_ITERATIONS, 20 )					;

	StaticReferenceTrajectory zeroReference;
	Controller controller( algorithm, zeroReference );
	ros::Time begin = ros::Time::now();

	// SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
	// ----------------------------------------------------------
	SimulationEnvironment sim( 0.0,2.5,process,controller );

	DVector xm(7);
	xm.setZero();

	sim.init( xm );
	sim.run( );

    ros::Time end = ros::Time::now();
    ros::Duration d = end-begin;
    ROS_WARN_STREAM("Time: " << d.toSec());

    // ... AND PLOT THE RESULTS
    // ------------------------
    VariablesGrid controls_, diffStates ;

    sim.getProcessDifferentialStates( diffStates );
    sim.getFeedbackControl(controls_);

    //algorithm.getControls(controls_);
    //algorithm.getDifferentialStates(diffStates);

	GnuplotWindow window;
		window.addSubplot(diffStates(0), "DIFFERENTIAL STATE  x");
	    window.addSubplot(diffStates(1),"DIFFERENTIAL STATE  y");
	    window.addSubplot(controls_,"CONTROL  u"   );
	window.plot( );


    std::cout<< controls_.getFirstVector() << std::endl;

  return controls_.getFirstVector();
  //////
}
*/

