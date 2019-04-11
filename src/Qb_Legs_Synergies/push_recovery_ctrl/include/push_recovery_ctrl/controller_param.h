// ------------------------------------------------------------ //
// Herein every numeric parameter for the push recovery controller
// is defined.
//
// Edoardo Sorelli: edoardo.sorelli@gmail.com
// ------------------------------------------------------------ //

#ifndef CONTROLLER_PARAM_H
#define CONTROLLER_PARAM_H

// standard variables
#define G           9.81f       // gravitational acceleration
#define DEG2RAD     3.1415/180  // conversion degree to radians
// robot dimensions
#define LEN_PELVIS  0.05        // [m] 
#define LEN_FEMUR   0.12        // [m]
#define LEN_TIBIA   0.18        // [m]
// Value initialization
#define V_INIT      0.0         // [m/s] 
#define L_INIT      0.05        // [m]
#define FH_INIT     0.03        // [m]
// Max control values
#define V_MAX       0.3         // [m/s]
#define L_MAX       0.2         // [m]
#define FH_MAX      0.1         // [m]
// parameters for the evaluation of the`setp length`
#define LEG_LENGTH  0.3f        // robot leg length
#define BETA        0.56f       // exponent of the definition of the nominal step length
// parameters ICP evaluation
#define COM_Z       0.2f        // approximate COM height
#define HALF_FOOT   0.05f       // security bound for stability analysis
// force sensor param
#define FZ_DETECT   5.0f       // offset to detect a contact force

#endif //CONTROLLER_PARAM_H
