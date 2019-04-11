// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------
// File:        qbotISS_ISS.cpp
//
// Description: Communication s-function for obtaining EMG of qbHand.
//              To be used with RS-485 on a Virtual COM.
//------------------------------------------------------------------------------


//==============================================================================
//                                                             main  definitions
//==============================================================================

#define S_FUNCTION_NAME  qbgetImuReadings
#define S_FUNCTION_LEVEL 2

//==============================================================================
//                                                                      includes
//==============================================================================

#include "definitions.h"
#include "simstruc.h"
#include "../../../qbAPI/src/qbmove_communications.h"
// #include <windows.h>

//==============================================================================
//                                                                   definitions
//==============================================================================

//===============================================================     parameters

#define params_qbot_id(i)     ( mxGetPr( ssGetSFcnParam( S, 0 ) )[ \
                                     i >= NUM_OF_QBOTS ? NUM_OF_QBOTS -1 : i ] )
#define N_IMU   ( (int)mxGetScalar( ssGetSFcnParam( S, 1 ) ) )
#define params_daisy_chaining  ( (bool)mxGetScalar( ssGetSFcnParam( S, 2 ) ) )
#define ACC_SF   ( (float)mxGetScalar( ssGetSFcnParam( S, 3 ) ) )
#define GYRO_SF   ( (float)mxGetScalar( ssGetSFcnParam( S, 4 ) ) )
#define MAG_SF   ( (float)mxGetScalar( ssGetSFcnParam( S, 5 ) ) )
#define PARAM_UNITY_ACC_FCN  ( (int) mxGetScalar( ssGetSFcnParam( S, 6 ) ) )
#define PARAM_UNITY_GYRO_FCN ( (int) mxGetScalar( ssGetSFcnParam( S, 7 ) ) )
#define PARAM_UNITY_MAG_FCN ( (int) mxGetScalar( ssGetSFcnParam( S, 8 ) ) )

#define G                       1
#define M_S2                    2
#define DEG_S                   1
#define RAD_S                   2
#define UT                    	1
#define DEG_TO_RAD             (3.14159265359 / 180.0)

//===================================================================     inputs

#define in_handle ( *(const HANDLE* *)ssGetInputPortSignal( S, 0 ) )[0]

//==================================================================     outputs

#define out(i)        ( ssGetOutputPortRealSignal       ( S, i ) )
                                     
#define out_handle_single ( (HANDLE* *)ssGetOutputPortSignal( S, 0 ) )[0]
#define out_handle_full   ( (HANDLE* *)ssGetOutputPortSignal( S, 2 ) )[0]

//==================================================================      dworks
#define dwork_out(i)      ( (real_T *)ssGetDWork( S, i ) )

//================================================================     constants

#define BUFFER_SIZES            15

comm_settings comm_settings_t; 
char n_imu = 0;                // number of connected imus
uint8_T qbot_id;                                // qbot id's
char* imus_info;
char* imu_table;
float* imus_magcal;
float* imus_sfs;
float* imu_values;
//=============================================================     enumerations

enum    QBOT_MODE { PRIME_MOVERS_POS    = 1, EQ_POS_AND_PRESET  = 2 };
enum    COMM_DIRS { RX = 1, TX = 2, BOTH = 3, NONE = 4 };

//===================================================================     macros

#define NUM_OF_QBOTS    ( (int)mxGetNumberOfElements( ssGetSFcnParam( S, 0 ) ) )
#define REF_A_WIDTH     ssGetInputPortWidth( S, 1 )
#define REF_B_WIDTH     ssGetInputPortWidth( S, 2 )
#define SIGN(x)         ( ( (x) < 0) ? -1 : ( (x) > 0 ) )

//==============================================================================
//                                                           function prototypes
//==============================================================================

unsigned char checksum_ ( unsigned char * buf, int size );
void    showOutputHandle( SimStruct *S );
// unsigned int verifychecksum_( unsigned char * buffer );

//==============================================================================
//                                                            mdlInitializeSizes
//==============================================================================
// The sizes information is used by Simulink to determine the S-function block's
// characteristics (number of inputs, outputs, states, etc.).
//==============================================================================

static void mdlInitializeSizes( SimStruct *S )
{
    int_T   status;                // for new type definition
    DTypeId COM_HANDLE_id;         // for new type definition
    HANDLE  handle_aux;            // for new type definition
    int i;                         // for cycles
    uint8_T qbot_id;                                // qbot id's
    uint8_T flags[3] = {0,0,0};
	uint8_T NUM_SF_PARAMS = 3;
	float* imu_values;
	
	comm_settings comm1;
	
    ssAllowSignalsWithMoreThan2D(S);
//======================================================     new type definition

    COM_HANDLE_id = ssRegisterDataType( S, "COM_HANDLE" );
    if( COM_HANDLE_id == INVALID_DTYPE_ID ) return;
    status = ssSetDataTypeSize( S, COM_HANDLE_id, sizeof(handle_aux) );
    if( status == 0)  return;
    status = ssSetDataTypeZero( S, COM_HANDLE_id, &handle_aux );
    if( status == 0 ) return;

    qbot_id = params_qbot_id(0);
    qbot_id = qbot_id <= 0   ? 1    : qbot_id;  // inferior limit
    qbot_id = qbot_id >= 128 ? 127  : qbot_id;  // superior limit
    
    
//==================================================== 
		
	#if defined(_WIN32) || defined(_WIN64)
        if(handle_aux == INVALID_HANDLE_VALUE) {
			printf("HHHH\n");
			return;
		}
    #else
		printf("CCCCCC\n");
        if(handle_aux == -1) {
			printf("GGGG\n");
			return;
		}
    #endif

	comm1.file_handle = handle_aux;
 
	
	//commGetNImu(&comm1, qbot_id, &n_imu);
    printf("Number of IMUs: %d\n", n_imu);
	
    // In this way, the sfunction reads number of imus from Mask
    n_imu = N_IMU;
	
		
	// Retrieve information on unities and scale factors
	imus_sfs = (float *) calloc(NUM_SF_PARAMS, sizeof(float));
//	commGetImusSFS(&comm_settings_t, qbot_id, imus_sfs);
	imus_sfs[0] = ACC_SF;
	imus_sfs[1] = GYRO_SF;
	imus_sfs[2] = MAG_SF;
		
	// Retrieve magnetometer calibration parameters
	imus_magcal = (float *) calloc(n_imu, 4*sizeof(float));
//	commGetMagParam(&comm1, qbot_id, imus_magcal, n_imu);
	// OGNI IMU HA I SUOI
	for (int i=0; i < n_imu; i++){
		imus_magcal[4*i + 1] = 1.0;
		imus_magcal[4*i + 2] = 1.0;
		imus_magcal[4*i + 3] = 1.0;
    }
	

//===================================================================     states

    ssSetNumContStates( S, 0 );
    ssSetNumDiscStates( S, 1 );

//===============================================================     parameters

    ssSetNumSFcnParams( S, 77 ); // 2 parameters:
                                //      - qbot I2C id
                                //      - N_IMU
                                //      - daisy chaining
						  // - acc_sf, gyro_sf, mag_sf, magcal_x, magcal_y, magcal_z
                          // - id, acc flag, gyro flag, mag flag (4 x 17)
//===================================================================     inputs

    if ( !ssSetNumInputPorts( S, 1 ) ) return;

/////////////////////////////////////// 0 ) pointer to HANDLE   ////////////////
    ssSetInputPortWidth             ( S, 0, 1 );
    ssSetInputPortDataType          ( S, 0, COM_HANDLE_id     );
    ssSetInputPortDirectFeedThrough ( S, 0, 1                 );
    ssSetInputPortRequiredContiguous( S, 0, 1                 );


//==================================================================     outputs
    imus_info = (char *) calloc(n_imu, 2*sizeof(char));
    imu_table = (char *) calloc(n_imu, 2*sizeof(char));
   
    // Read Imus Table				
	/*
    commGetImusMode(&comm_settings_t, qbot_id, imus_info, n_imu);
    for (int i=0; i < n_imu; i++){
        printf("%d - %x\n", imus_info[2*i + 0], imus_info[2*i + 1]);		
        imu_table[2*i + 0] = imus_info[2*i];
		imu_table[2*i + 1] = imus_info[2*i+1];
    } */
    /*
    imus_info[0] = 0;
    imus_info[1] = 6;
    imus_info[2] = 1;
    imus_info[3] = 3;
 */   
    // Read Imus Table from Mask
    for (int i = 0; i < n_imu; i++) {
        imus_info[2*i] = ( (int) mxGetScalar( ssGetSFcnParam( S, 9+4*i ) ) );
        
        imus_info[2*i+1] =  0;
        if (( (bool) mxGetScalar( ssGetSFcnParam( S, 9+4*i + 1 ) ) )) // Acc flag
            imus_info[2*i+1] += 4;
        if (( (bool) mxGetScalar( ssGetSFcnParam( S, 9+4*i + 2 ) ) )) // Gyro flag
            imus_info[2*i+1] += 2;
        if (( (bool) mxGetScalar( ssGetSFcnParam( S, 9+4*i + 3 ) ) )) // Mag flag
            imus_info[2*i+1] += 1;
    }

    for (int i=0; i < n_imu; i++){
       // printf("%d - %x\n", imus_info[2*i + 0], imus_info[2*i + 1]);		
        imu_table[2*i + 0] = imus_info[2*i];
		imu_table[2*i + 1] = imus_info[2*i+1];
    }

	
	if(params_daisy_chaining)
	{
		if (!ssSetNumOutputPorts(S, 1 + n_imu)) return;

///////////////////////////////// 3 ) com handle    ////////////////////////////
		ssSetOutputPortWidth   ( S, n_imu, 1             );
		ssSetOutputPortDataType( S, n_imu, COM_HANDLE_id );
	}
	else
	{
		if (!ssSetNumOutputPorts(S, n_imu)) return;
	}

////////////////////////////////   values      //////////////////////////////
	for (int i=0; i < n_imu; i++){
		
		flags[0] = flags[1] = flags[2] = 0;
		if (imu_table[2*i+1] & 0x04) flags[0] = 1;
		if (imu_table[2*i+1] & 0x02) flags[1] = 1;
		if (imu_table[2*i+1] & 0x01) flags[2] = 1;
	
//        ssSetOutputPortWidth    ( S, i, NUM_OF_QBOTS * 6 * (flags[0]+flags[1]+flags[2]) );
		
		DECL_AND_INIT_DIMSINFO(di);
		int_T dims[2];

		di.numDims = 2;
		dims[0] = NUM_OF_QBOTS;
		dims[1] = 3 * 3; //(flags[0]+flags[1]+flags[2]);
		di.dims = dims;
		di.width = NUM_OF_QBOTS * 3 * 3; //(flags[0]+flags[1]+flags[2]);
		ssSetOutputPortDimensionInfo(S, i, &di);

		ssSetOutputPortDataType ( S, i, SS_DOUBLE );
	}

//=============================================================     sample times

    ssSetNumSampleTimes(S, 1);

//=============================================================     work vectors

    ssSetNumDWork(S, NUM_OF_QBOTS);     // 0 dwork vector elements
    ssSetNumRWork(S, 0);                // 0 real work vector elements
    ssSetNumIWork(S, 0);                // 0 work vector elements
    ssSetNumPWork(S, 0);                // 0 pwork vector elements:
    ssSetNumModes(S, 0);                // 0 mode work vector elements
    ssSetNumNonsampledZCs(S, 0);        // 0 nonsampled zero crossings

    for( i = 0; i < NUM_OF_QBOTS; ++i)
    {
        ssSetDWorkWidth(S, i, 17*6);
        ssSetDWorkDataType(S, i, SS_DOUBLE);
    }

//===================================================================     others

    ssSetOptions(S, SS_OPTION_ALLOW_INPUT_SCALAR_EXPANSION);

}

//==============================================================================
//                                                          mdlSetInputPortWidth
//==============================================================================
// This method is called with the candidate width for a dynamically sized port.
// If the proposed width is acceptable, the actual port width is set using
// ssSetInputPortWidth.
//==============================================================================

#undef MDL_SET_INPUT_PORT_WIDTH   // Change to #undef to remove function
#if defined(MDL_SET_INPUT_PORT_WIDTH) && defined(MATLAB_MEX_FILE)
static void mdlSetInputPortWidth( SimStruct *S, int portIndex, int width )
{
    switch( portIndex )
    {
        case 0:
            ssSetInputPortWidth( S, portIndex, 1 );
        break;
        default:
            ssSetInputPortWidth( S, portIndex, width );
        break;
    }
}
#endif /* MDL_SET_INPUT_PORT_WIDTH */

//==============================================================================
//                                                         mdlSetOutputPortWidth
//==============================================================================
// This method is called with the candidate width for a dynamically sized port.
// If the proposed width is acceptable, the actual port width is set using
// ssSetOutputPortWidth.
//==============================================================================

#undef MDL_SET_OUTPUT_PORT_WIDTH   // Change to #undef to remove function
#if defined(MDL_SET_OUTPUT_PORT_WIDTH) && defined(MATLAB_MEX_FILE)
static void mdlSetOutputPortWidth( SimStruct *S, int portIndex, int width )
{
        ssSetOutputPortWidth( S, portIndex, NUM_OF_QBOTS );
}
#endif /* MDL_SET_OUTPUT_PORT_WIDTH */

//==============================================================================
//                                                      mdlInitializeSampleTimes
//==============================================================================
// This function is used to specify the sample time(s) for your S-function.
// o  The sample times are specified as pairs "[sample_time, offset_time]" via
//    the following macros:
//    ssSetSampleTime(S, sampleTimePairIndex, sample_time)
//    ssSetOffsetTime(S, offsetTimePairIndex, offset_time)
//    Where sampleTimePairIndex starts at 0.
// o  A discrete function that changes at a specified rate should register the
//    discrete sample time pair
//          [discrete_sample_period, offset]
//    where
//          discrete_sample_period > 0.0
//    and
//          0.0 <= offset < discrete_sample_period
//==============================================================================

static void mdlInitializeSampleTimes( SimStruct *S )
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

//==============================================================================
//                                                                      mdlStart
//==============================================================================
// This function is called once at start of model execution.
//==============================================================================

#define MDL_START  // Change to #undef to remove function
#if     defined(MDL_START)
static void mdlStart( SimStruct *S )
{
}
#endif /* MDL_START */

//==============================================================================
//                                                                    mdlOutputs
//==============================================================================
// Values are assigned to requested outputs in this function.
//==============================================================================

static void mdlOutputs( SimStruct *S, int_T tid )
{
    short int val;
    uint8_T qbot_id;                                // qbot id's
    char* values;
    int c = 0;
    int c_s = 0;
	int c_id;
	uint8_T flags[3] = {0,0,0};
    int imu_data_size;
    int i;

//=============================     should an output handle appear in the block?

    if(params_daisy_chaining) showOutputHandle(S);

//====================================================     should we keep going?

    #if defined(_WIN32) || defined(_WIN64)
        if(in_handle == INVALID_HANDLE_VALUE) return;
    #else
        if(in_handle == -1) return;
    #endif


//==========================================     asking imu reading

    comm_settings_t.file_handle = in_handle;

    for(i = 0; i < NUM_OF_QBOTS; i++)
    {
//============================================================      qbot ID check

     qbot_id = params_qbot_id(i);
     qbot_id = qbot_id <= 0   ? 1    : qbot_id;  // inferior limit
     qbot_id = qbot_id >= 128 ? 127  : qbot_id;  // superior limit
     
	 // Imu values is a 3 sensors x 3 axes x n_imu values
	imu_values = (float *) calloc(n_imu, 3*3*sizeof(float));
  
    commGetImuReadings(&comm_settings_t, qbot_id, imu_table, imus_magcal, imus_sfs, n_imu, imu_values);
			
	for (i = 0; i < n_imu; i++) {
		
	//	printf("IMU: %d\n", imu_table[2*i + 0]);
		for (int k=0; k<9; k++)
			out(i)[k] = (float)imu_values[3*3*i+k];
	/*	
		if (imu_table[2*i + 1] & 0x04){
			printf("Accelerometer\n");
			printf("%f, %f, %f\n", imu_values[3*3*i], imu_values[3*3*i+1], imu_values[3*3*i+2]);
		}
		if (imu_table[2*i + 1] & 0x02){
			printf("Gyroscope\n");
			printf("%f, %f, %f\n", imu_values[3*3*i+3], imu_values[3*3*i+4], imu_values[3*3*i+5]);
		}
		if (imu_table[2*i + 1] & 0x01){
			printf("Magnetometer\n");
			printf("%f, %f, %f\n", imu_values[3*3*i+6], imu_values[3*3*i+7], imu_values[3*3*i+8]);
		}
	*/	
	}
	
	}

}


//==============================================================================
//                                                                     mdlUpdate
//==============================================================================
// This function is called once for every major integration time step.
//==============================================================================

#define MDL_UPDATE  // Change to #undef to remove function
#if defined(MDL_UPDATE)
static void mdlUpdate( SimStruct *S, int_T tid )
{

}
#endif /* MDL_UPDATE */

//==============================================================================
//                                                                  mdlTerminate
//==============================================================================
// In this function, you should perform any actions that are necessary at the
// termination of a simulation.
//==============================================================================

static void mdlTerminate( SimStruct *S )
{
    char aux_char;
    int try_counter;
    int i;
    comm_settings comm_settings_t;

    comm_settings_t.file_handle = in_handle;

    if (comm_settings_t.file_handle == INVALID_HANDLE_VALUE) {
        closeRS485(&comm_settings_t);
        return;
    }
 
    closeRS485(&comm_settings_t);
}

//==============================================================================
//                                                              showOutputHandle
//==============================================================================
// TODO
//==============================================================================

void showOutputHandle( SimStruct *S )
{
        out_handle_full     = (HANDLE *) &in_handle;    // appear in output 3
}

//==============================================================================
//                                                   Required S-function trailer
//==============================================================================

#ifdef  MATLAB_MEX_FILE    // Is this file being compiled as a MEX-file?
#include "simulink.c"      // MEX-file interface mechanism
#else
#include "cg_sfun.h"       // Code generation registration function
#endif