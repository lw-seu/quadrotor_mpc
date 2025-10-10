#include <memory>
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

int main()
{
    USING_NAMESPACE_ACADO

    const bool CODE_GEN = true;

    // Variables 
    DifferentialState     p_x, p_y, p_z;
    DifferentialState     q_w, q_x, q_y, q_z;
    DifferentialState     v_x, v_y, v_z;
    Control               w_x, w_y, w_z, T;   
    OnlineData            d_x, d_y, d_z;


    // Parameters with exemplary values. These are set/overwritten at runtime.
    const double t_start = 0.0;     // Initial time [s]
    const double t_end = 2.0;       // Time horizon [s]
    const double dt = 0.1;          // Discretization time [s]
    const int N = round(t_end/dt);  // Number of nodes
    const double g_z = 9.8066;      // Gravity is everywhere [m/s^2]
    const double w_max_yaw = 2.0;     // Maximal yaw rate [rad/s]
    const double w_max_xy = 1.5;      // Maximal pitch and roll rate [rad/s]
    const double T_min = 0;         // Minimal thrust [N/kg]
    const double m = 1.5;           // iris mass [kg]
    const double T_max = 2.0*g_z*m;        // Maximal thrust [N/kg]
    
    // Model equations
    DifferentialEquation f; 
    f << dot(p_x) ==  v_x;
    f << dot(p_y) ==  v_y;
    f << dot(p_z) ==  v_z;
    f << dot(q_w) ==  0.5 * ( - w_x * q_x - w_y * q_y - w_z * q_z);
    f << dot(q_x) ==  0.5 * ( w_x * q_w + w_z * q_y - w_y * q_z);
    f << dot(q_y) ==  0.5 * ( w_y * q_w - w_z * q_x + w_x * q_z);
    f << dot(q_z) ==  0.5 * ( w_z * q_w + w_y * q_x - w_x * q_y);
    f << dot(v_x) ==  2 * ( q_w * q_y + q_x * q_z ) * T/m + d_x;
    f << dot(v_y) ==  2 * ( q_y * q_z - q_w * q_x ) * T/m + d_y;
    f << dot(v_z) ==  ( 1 - 2 * q_x * q_x - 2 * q_y * q_y ) * T/m - g_z + d_z;

    // Reference functions and weighting matrices
    Function h, hN;
    // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
    // Running cost vector consists of all states and inputs.
    h << p_x << p_y << p_z
    << q_w << q_x << q_y << q_z
    << v_x << v_y << v_z 
    << w_x << w_y << w_z << T;   

    // End cost vector consists of all states (no inputs at last state).
    hN << p_x << p_y << p_z
    << q_w << q_x << q_y << q_z 
    << v_x << v_y << v_z;

    DMatrix Q(h.getDim(), h.getDim());
    Q.setIdentity();
    Q(0,0) = 200;   // x
    Q(1,1) = 200;   // y
    Q(2,2) = 500;   // z
    Q(3,3) = 50;   // qw
    Q(4,4) = 50;   // qx
    Q(5,5) = 50;   // qy
    Q(6,6) = 50;   // qz
    Q(7,7) = 10;   // vx
    Q(8,8) = 10;   // vy
    Q(9,9) = 10;   // vz
    Q(10,10) = 1;   // wx
    Q(11,11) = 1;   // wy
    Q(12,12) = 1;   // wz
    Q(13,13) = 1;   // T

    // End cost weight matrix
    DMatrix QN(hN.getDim(), hN.getDim());
    QN.setIdentity();
    QN(0,0) = Q(0,0);   // x
    QN(1,1) = Q(1,1);   // y
    QN(2,2) = Q(2,2);   // z
    QN(3,3) = Q(3,3);   // qw
    QN(4,4) = Q(4,4);   // qx
    QN(5,5) = Q(5,5);   // qy
    QN(6,6) = Q(6,6);   // qz
    QN(7,7) = Q(7,7);   // vx
    QN(8,8) = Q(8,8);   // vy
    QN(9,9) = Q(9,9);   // vz

    // Set a reference for the analysis (if CODE_GEN is false).
    // Reference is at x = 2.0m in hover (qw = 1).
    DVector r(h.getDim());    // Running cost reference
    r.setZero();
    r(0) = 2.0;
    r(1) = 2.0;
    r(2) = 2.0;
    r(3) = 0.7071;
    r(6) = 0.7071;
    r(10) = g_z;

    DVector rN(hN.getDim());   // End cost reference
    rN.setZero();
    rN(0) = r(0);
    rN(1) = r(1);
    rN(2) = r(2);
    rN(3) = r(3);
    rN(6) = r(6);

    OCP ocp( t_start, t_end, N );

    if(!CODE_GEN)
    {
        // For analysis, set references.
        ocp.minimizeLSQ( Q, h, r );
        ocp.minimizeLSQEndTerm( QN, hN, rN );
    }else{
        // For code generation, references are set during run time.
        BMatrix Q_sparse(h.getDim(), h.getDim());
        Q_sparse.setIdentity();

        BMatrix QN_sparse(hN.getDim(), hN.getDim());
        QN_sparse.setIdentity();

        ocp.minimizeLSQ( Q_sparse, h);
        ocp.minimizeLSQEndTerm( QN_sparse, hN );
    }

    // Add system dynamics
    ocp.subjectTo( f );
    // Add constraints
    ocp.subjectTo(-w_max_xy <= w_x <= w_max_xy);
    ocp.subjectTo(-w_max_xy <= w_y <= w_max_xy);
    ocp.subjectTo(-w_max_yaw <= w_z <= w_max_yaw);
    ocp.subjectTo( T_min <= T <= T_max);

    ocp.setNOD(3);

    if(!CODE_GEN)
    {
        // Set initial state
        ocp.subjectTo( AT_START, p_x ==  0.0 );
        ocp.subjectTo( AT_START, p_y ==  0.0 );
        ocp.subjectTo( AT_START, p_z ==  0.0 );
        ocp.subjectTo( AT_START, q_w ==  1.0 );
        ocp.subjectTo( AT_START, q_x ==  0.0 );
        ocp.subjectTo( AT_START, q_y ==  0.0 );
        ocp.subjectTo( AT_START, q_z ==  0.0 );
        ocp.subjectTo( AT_START, v_x ==  0.0 );
        ocp.subjectTo( AT_START, v_y ==  0.0 );
        ocp.subjectTo( AT_START, v_z ==  0.0 );
        ocp.subjectTo( AT_START, w_x ==  0.0 );
        ocp.subjectTo( AT_START, w_y ==  0.0 );
        ocp.subjectTo( AT_START, w_z ==  0.0 );

        // Setup some visualization
        GnuplotWindow window1( PLOT_AT_EACH_ITERATION );
        window1.addSubplot( p_x,"position x" );
        window1.addSubplot( p_y,"position y" );
        window1.addSubplot( p_z,"position z" );
        window1.addSubplot( q_w,"quaternion w" );
        window1.addSubplot( q_x,"quaternion x" );
        window1.addSubplot( q_y,"quaternion y" );
        window1.addSubplot( q_z,"quaternion z" );
        window1.addSubplot( v_x,"verlocity x" );
        window1.addSubplot( v_y,"verlocity y" );
        window1.addSubplot( v_z,"verlocity z" );

        GnuplotWindow window3( PLOT_AT_EACH_ITERATION );
        window3.addSubplot( w_x,"rotation-acc x" );
        window3.addSubplot( w_y,"rotation-acc y" );
        window3.addSubplot( w_z,"rotation-acc z" ); 
        window3.addSubplot( T,"Thrust" );


        // Define an algorithm to solve it.
        OptimizationAlgorithm algorithm(ocp);
        algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
        algorithm.set( KKT_TOLERANCE, 1e-3 );
        algorithm << window1;
        algorithm << window3;
        algorithm.solve();

    }

    else
    {
        OCPexport mpc(ocp);

        mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
        mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
        mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
        mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
        mpc.set(NUM_INTEGRATOR_STEPS,   N);
        mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
        mpc.set(HOTSTART_QP,            YES);
        mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
        mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
        mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
        mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision

        // Do not generate tests, makes or matlab-related interfaces.
        mpc.set( GENERATE_TEST_FILE,          YES);
        mpc.set( GENERATE_MAKE_FILE,          NO);
        mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
        mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

        // Finally, export everything.
        if(mpc.exportCode("quadrotor_mpc_export") != SUCCESSFUL_RETURN)
            exit( EXIT_FAILURE );
        mpc.printDimensionsQP( );
    }
    
    return EXIT_SUCCESS;
}