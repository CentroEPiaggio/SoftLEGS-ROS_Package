//=================================================================     includes
#include <globals.h>

//=============================================      global variables definition


struct st_ref   g_ref;                  // motor variables
struct st_meas  g_meas;                 // measurements
struct st_data  g_rx;                   // income data
struct st_mem   g_mem, c_mem;           // memory
struct st_dev   device;                 // device related variables
struct st_calib calib;

int32 opened_hand_pos;
int32 closed_hand_pos;
int8 dx_sx_hand;            //-1 dx, 1 sx

float tau_feedback;

// utility timer value for display

uint32 timer_value;

uint8 reset_last_value_flag;
