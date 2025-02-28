#include "mex.h"
#include <math.h>

/* Define constants */
#define GRAV 9.81
#define AIR_DENSITY 1.225
#define CANOPY_AREA 10.094
#define MASS 34.879

/* Input Arguments */
#define T_IN prhs[0]
#define STATES_IN prhs[1]
#define INPUTS_IN prhs[2]
#define C_L_0_IN prhs[3]
#define C_L_SYM_DEFLECTION_IN prhs[4]
#define C_D_0_IN prhs[5]
#define C_D_SYM_DEFLECTION_IN prhs[6]
#define T_ROLL_IN prhs[7]
#define K_ROLL_IN prhs[8]

/* Output Arguments */
#define DX_OUT plhs[0]
#define Y_OUT plhs[1]

void FourDOF_ODE_C_MEX(double t, double *states, double *inputs, double C_L_0, double C_L_SYM_DEFLECTION, double C_D_0, double C_D_SYM_DEFLECTION, double T_ROLL, double K_ROLL, double *dx, double *y) {
    /* States */
    double v_x = states[0];
    double yaw = states[1];
    double v_z = states[2];
    double roll = states[3];

    /* Inputs */
    double delta_asym = inputs[0];
    double delta_sym = inputs[1];

    /* Some variables needed for dynamic equations */
    double alpha = atan2(v_z, v_x);
    double airspeed = sqrt(v_x * v_x + v_z * v_z);

    double lift = 0.5 * AIR_DENSITY * airspeed * airspeed * CANOPY_AREA * (C_L_0 + C_L_SYM_DEFLECTION * delta_sym);
    double drag = 0.5 * AIR_DENSITY * airspeed * airspeed * CANOPY_AREA * (C_D_0 + C_D_SYM_DEFLECTION * delta_sym);

    /* Dynamic equations */
    double roll_dot = 1 / T_ROLL * (K_ROLL * delta_asym - roll);
    double yaw_dot = GRAV / v_x * tan(roll) + (v_z * roll_dot) / (v_x * cos(roll));

    double v_x_dot = 1 / MASS * (lift * sin(alpha) - drag * cos(alpha)) - v_z * yaw_dot * sin(roll);
    double v_z_dot = 1 / MASS * (-lift * cos(alpha) - drag * sin(alpha)) + GRAV * cos(roll) + v_x * yaw_dot * sin(roll);

    /* Return state derivatives */
    dx[0] = v_x_dot;
    dx[1] = yaw_dot;
    dx[2] = v_z_dot;
    dx[3] = roll_dot;

    /* Return states */
    y[0] = v_x;
    y[1] = yaw;
    y[2] = v_z;
    y[3] = roll;
}

/* The gateway function */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    double t;
    double *states;
    double *inputs;
    double C_L_0;
    double C_L_SYM_DEFLECTION;
    double C_D_0;
    double C_D_SYM_DEFLECTION;
    double T_ROLL;
    double K_ROLL;
    double *dx;
    double *y;

    /* Check for proper number of arguments */
    if (nrhs != 10) {
        mexErrMsgIdAndTxt("FourDOF_ODE_C_MEX:nrhs", "Nine inputs required. %d" , nrhs);
    }
    if (nlhs != 2) {
        mexErrMsgIdAndTxt("FourDOF_ODE_C_MEX:nlhs", "Two outputs required.");
    }

    /* Get the inputs */
    t = mxGetScalar(T_IN);
    states = mxGetPr(STATES_IN);
    inputs = mxGetPr(INPUTS_IN);
    C_L_0 = mxGetScalar(C_L_0_IN);
    C_L_SYM_DEFLECTION = mxGetScalar(C_L_SYM_DEFLECTION_IN);
    C_D_0 = mxGetScalar(C_D_0_IN);
    C_D_SYM_DEFLECTION = mxGetScalar(C_D_SYM_DEFLECTION_IN);
    T_ROLL = mxGetScalar(T_ROLL_IN);
    K_ROLL = mxGetScalar(K_ROLL_IN);

    /* Create the output matrices */
    DX_OUT = mxCreateDoubleMatrix(4, 1, mxREAL);
    Y_OUT = mxCreateDoubleMatrix(4, 1, mxREAL);

    /* Get pointers to the real data in the output matrices */
    dx = mxGetPr(DX_OUT);
    y = mxGetPr(Y_OUT);

    /* Call the computational routine */
    FourDOF_ODE_C_MEX(t, states, inputs, C_L_0, C_L_SYM_DEFLECTION, C_D_0, C_D_SYM_DEFLECTION, T_ROLL, K_ROLL, dx, y);
}
