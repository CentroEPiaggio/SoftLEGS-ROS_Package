//=================================================================     includes
#include <interruptions.h>
#include <command_processing.h>
#include <globals.h>


//==============================================================================
//                                                            RS485 RX INTERRUPT
//==============================================================================
// Processing RS-485 data frame:
//
// - 0:     Waits for beggining characters
// - 1:     Waits for ID;
// - 2:     Data length;
// - 3:     Receive all bytes;
// - 4:     Wait for another device end of transmission;
//
//==============================================================================

CY_ISR(ISR_RS485_RX_ExInterrupt){

//===============================================     local variables definition

    
    static uint8    state = 0;                          // actual state
    static struct   st_data data_packet;                // local data packet
    static uint8    rx_queue[3];                        // last 3 bytes received
    static uint8    rx_data;                            // RS485 UART rx data
    static uint8    rx_data_type;                       // packet for me or not
    extern int status;


//==========================================================     receive routine

// get data while rx fifo is not empty
    while (UART_RS485_ReadRxStatus() & UART_RS485_RX_STS_FIFO_NOTEMPTY) {
       rx_data = UART_RS485_GetChar();

        switch (state) {
            // ----- wait for frame start -----
            case 0:

                rx_queue[0] = rx_queue[1];
                rx_queue[1] = rx_queue[2];
                rx_queue[2] = rx_data;

                // Finding starting frame
                if ((rx_queue[1] == ':') && (rx_queue[2] == ':')) {
                    rx_queue[0] = 0;
                    rx_queue[1] = 0;
                    rx_queue[2] = 0;
                    state       = 1;
                } else if ((rx_queue[0] == 63) && (rx_queue[1] == 13) && (rx_queue[2] == 10)){
                    // 63 = ASCII - ?; 13 = ASCII - CR; 10 = ASCII - LF 
                    //UART_RS485_PutString("Ciao");
                    //infoSend();
                } else if (rx_queue[0] == 60){
                    //else if ((rx_queue[0] == 60) && (rx_queue[1] == 13) && (rx_queue[2] == 10)){
                    // 60 = ASCII - <; 13 = ASCII - CR; 10 = ASCII - LF 
                    
                    //UART_RS485_PutString("Ciao");
                    status = 1;
                } else if ((rx_queue[0] == 62) && (rx_queue[1] == 13) && (rx_queue[2] == 10)){
                    // 62 = ASCII - >; 13 = ASCII - CR; 10 = ASCII - LF 
                    status = 0;
                }
                break;

            // ----- wait for id -----
            case 1:

                // packet is for my ID or is broadcast
                if((rx_data == c_mem.id) || (rx_data == 0)) {
                    rx_data_type = 0;
                } else {                //packet is for others
                    rx_data_type = 1;
                }
                data_packet.length = -1;
                state = 2;
                break;

            // ----- wait for length -----
            case 2:

                data_packet.length = rx_data;
                // check validity of pack length
               if (data_packet.length <= 1) {
                    data_packet.length = -1;
                    state = 0;
                } else if (data_packet.length > 128) {
                    data_packet.length = -1;
                    state = 0;
                } else {
                    data_packet.ind = 0;
                    if(rx_data_type == 0) {
                        state = 3;          // packet for me or broadcast
                    } else {
                        state = 4;          // packet for others
                    }
                }
                break;

            // ----- receving -----
            case 3:

                data_packet.buffer[data_packet.ind] = rx_data;
                data_packet.ind++;

                // check end of transmission
                if (data_packet.ind >= data_packet.length) {
                    // verify if frame ID corresponded to the device ID
                    if (rx_data_type == 0) {
                        // copying data from buffer to global packet
                        memcpy(g_rx.buffer, data_packet.buffer, data_packet.length);
                        g_rx.length = data_packet.length;
                        //commProcess();
                    }
                    data_packet.ind    =  0;
                    data_packet.length = -1;
                    state              =  0;
                }
                break;

            // ----- other device is receving -----
            case 4:

//                if(!(--data_packet.length)) {
//                    data_packet.ind    = 0;
//                    data_packet.length = -1;
//                    RS485_CTS_Write(1);             //CTS on falling edge
//                    RS485_CTS_Write(0);
//                    state              = 0;
//                }
                break;
        }
    }

    /* PSoC3 ES1, ES2 RTC ISR PATCH  */
    #if(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3)
        #if((CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_3A_ES2) && (ISR_RS485_RX__ES2_PATCH ))
//            ISR_MOTORS_CONTROL_ISR_PATCH();
        #endif
    #endif
}

//==============================================================================
//                                                            FUNCTION SCHEDULER
//==============================================================================
// Call all the function with the right frequency
//==============================================================================

//void function_scheduler(void) {
    // Base frequency 1000 Hz

    //static uint16 counter_calibration = DIV_INIT_VALUE;
    // QUESTE VANNO RISCRITTE PER I SEGNALI EMG
    //analog_read_init(0);
    //analog_read_end(0);
    //analog_read_init(1);
    //analog_read_end(1);

    // Divider 10, freq = 500 Hz
    //if (calib.enabled == TRUE) {
    //    if (counter_calibration == CALIBRATION_DIV) {
    //        calibration();
    //        counter_calibration = 0;
    //    }
    //    counter_calibration++;
    //}

    //timer_value = (uint32)MY_TIMER_ReadCounter();
//}




//==============================================================================
//                                                           ANALOG MEASUREMENTS
//==============================================================================

//void analog_read_init(uint8 index) {

    // should I execute the function for this index?
//    if(index >= NUM_OF_ANALOG_INPUTS)
//        return;

//    AMUX_FastSelect(index);
//    ADC_StartConvert();
//}


//void analog_read_end(uint8 index) {

//    static int32 value, i_aux;

//    static uint16 emg_counter_1 = 0;
//    static uint16 emg_counter_2 = 0;

//    static uint8 emg_1_status = 1;  // 0 normal execution
//    static uint8 emg_2_status = 1;  // 1 reset status
                                    // 2 discard values
                                    // 3 sum values and mean
                                    // 4 wait

    // should I execute the function for this index?
//    if(index >= NUM_OF_ANALOG_INPUTS)
//        return;

//    if (ADC_IsEndConversion(ADC_WAIT_FOR_RESULT)) {

//        value = (int32) ADC_GetResult16();
//        ADC_StopConvert();

//        switch(index) {
            // --- Input tension ---
//            case 0:
//                device.tension = filter_v((value - 1638) * device.tension_conv_factor);
                //until there is no valid input tension repeat this measurement
//                if (device.tension < 0) {
//                    emg_1_status = 1;   // reset status
//                    emg_2_status = 1;
//                    device.tension_valid = FALSE;

//                    if (c_mem.emg_calibration_flag) {
//                        if ((c_mem.input_mode == INPUT_MODE_EMG_PROPORTIONAL) ||
//                            (c_mem.input_mode == INPUT_MODE_EMG_INTEGRAL) ||
//                            (c_mem.input_mode == INPUT_MODE_EMG_FCFS) ||
//                            (c_mem.input_mode == INPUT_MODE_EMG_FCFS_ADV)) {
//                            g_ref.onoff = 0x00;
//                            MOTOR_ON_OFF_Write(g_ref.onoff);
//                        }
//                    }
//                } else {
//                    device.tension_valid = TRUE;
//                    if(g_mem.activate_pwm_rescaling)        //pwm rescaling is activated
//                        pwm_limit_search();                 //only for 12V motors
//                }
//                break;

            // --- Current motor 1 ---
//            case 1:
//                if (device.tension_valid) {
//                    g_meas.curr[0] =  filter_i1(abs(((value - 1638) * 4000) / (1638)));
//                } else {
//                    g_meas.curr[0] = 0;
//                }
//                break;

            // --- EMG 1 ---
//            case 2:
                // execute only if there is tension
//                if (device.tension_valid == FALSE) {
//                    g_meas.emg[0] = 0;
//                    break;
//                }

                // if calibration is not needed go to "normal execution"
 //               if (!g_mem.emg_calibration_flag) {
 //                   emg_1_status = 0; //normal execution
 //               }
                // EMG 1 calibration state machine
 //               switch(emg_1_status) {
 //                   case 0: // normal execution
 //                       i_aux = filter_ch1(value);
 //                       i_aux = (1024 * i_aux) / g_mem.emg_max_value[0];

                        //Saturation
//                        if (i_aux < 0) {
//                            i_aux = 0;
//                        } else if (i_aux > 1024) {
//                            i_aux = 1024;
//                        }

//                        g_meas.emg[0] = i_aux;
//                        break;

//                    case 1: // reset variables
//                        emg_counter_1 = 0;
//                        g_mem.emg_max_value[0] = 0;
//                        emg_1_status = 2; // goto next status
//                        break;

//                    case 2: // discard first EMG_SAMPLE_TO_DISCARD samples
//                        emg_counter_1++;
//                        if (emg_counter_1 == EMG_SAMPLE_TO_DISCARD) {
//                            emg_counter_1 = 0;          // reset counter
//                            LED_REG_Write(0x01);        // turn on LED
//                            emg_1_status = 3;           // sum and mean status
//                        }
//                        break;

//                    case 3: // sum first SAMPLES_FOR_EMG_MEAN samples
                        // NOTE max(value)*SAMPLES_FOR_EMG_MEAN must fit in 32bit
//                        emg_counter_1++;
//                        g_mem.emg_max_value[0] += filter_ch1(value);
//                        if (emg_counter_1 == SAMPLES_FOR_EMG_MEAN) {
//                            g_mem.emg_max_value[0] = g_mem.emg_max_value[0] / SAMPLES_FOR_EMG_MEAN; // calc mean
//                            LED_REG_Write(0x00);        // led OFF
//                            emg_counter_1 = 0;          // reset counter
//                            emg_1_status = 0;           // goto normal execution
//                        }
//                        break;

//                    default:
//                        break;
//                }
//                break; // main switch break

            // --- EMG 2 ---
//            case 3:
                // execute only if there is tension
//                if (device.tension_valid == FALSE) {
//                    g_meas.emg[1] = 0;
//                    break;
//                }

                // if calibration is not needed go to "normal execution"
//                if (!g_mem.emg_calibration_flag) {
//                    emg_2_status = 0; // normal execution
//                }

                // EMG 2 calibration state machine
//                switch(emg_2_status) {
//                    case 0: // normal execution
//                        i_aux = filter_ch2(value);
//                        i_aux = (1024 * i_aux) / g_mem.emg_max_value[1];

//                        if (i_aux < 0) {
//                            i_aux = 0;
//                        } else if (i_aux > 1024) {
//                            i_aux = 1024;
//                        }

//                        g_meas.emg[1] = i_aux;
//                        break;

//                    case 1: // reset variables
//                        emg_counter_2 = 0;
//                        g_mem.emg_max_value[1] = 0;
//                        emg_2_status = 4; // wait for EMG 1 calibration
//                        break;

//                    case 2: // discard first EMG_SAMPLE_TO_DISCARD samples
//                        emg_counter_2++;
//                        if (emg_counter_2 == EMG_SAMPLE_TO_DISCARD) {
//                            emg_counter_2 = 0;          // reset counter
//                            LED_REG_Write(0x01);        // turn on LED
//                            emg_2_status = 3;           // sum and mean status
//                        }
//                        break;

//                    case 3: // sum first SAMPLES_FOR_EMG_MEAN samples
                        // NOTE max(value)*SAMPLES_FOR_EMG_MEAN must fit in 32bit
//                        emg_counter_2++;
//                        g_mem.emg_max_value[1] += filter_ch2(value);
//                        if (emg_counter_2 == SAMPLES_FOR_EMG_MEAN) {
//                            g_mem.emg_max_value[1] = g_mem.emg_max_value[1] / SAMPLES_FOR_EMG_MEAN; // calc mean
//                            LED_REG_Write(0x00);        // led OFF
//                            emg_counter_2 = 0;          // reset counter

                            // if EMG control mode active, activate motors accordingly with startup value
//                            if ((c_mem.input_mode == INPUT_MODE_EMG_PROPORTIONAL) ||
//                                (c_mem.input_mode == INPUT_MODE_EMG_INTEGRAL) ||
//                                (c_mem.input_mode == INPUT_MODE_EMG_FCFS) ||
//                                (c_mem.input_mode == INPUT_MODE_EMG_FCFS_ADV)) {
//                                if (c_mem.control_mode == CONTROL_ANGLE) {
//                                    g_ref.pos[0] = g_meas.pos[0];
//                                    g_ref.pos[1] = g_meas.pos[1];
//                                }
//                                g_ref.onoff = c_mem.activ;
//                                MOTOR_ON_OFF_Write(g_ref.onoff);
//                            }

//                            emg_2_status = 0;           // goto normal execution
//                        }
//                        break;

//                    case 4: // wait for EMG calibration to be done
//                        if (emg_1_status == 0) {
//                            emg_2_status = 2;           // goto discart sample
//                        }
//                        break;

//                    default:
//                        break;
//                }
//                break; // emg switch break

//            default:
//                break; // main switch break
//        }
//    }
//}

//==============================================================================
//                                                           OVERCURRENT CONTROL
//==============================================================================

//void overcurrent_control(void) {
//    if (c_mem.current_limit != 0) {
        // if the current is over the limit
//        if (g_meas.curr[0] > c_mem.current_limit) {
            //decrese pwm_limit
//            device.pwm_limit--;
        // if the current is in the safe zone
//        } else if (g_meas.curr[0] < (c_mem.current_limit - CURRENT_HYSTERESIS)) {
            //increase pwm_limit
//            device.pwm_limit++;
//        }

        // bound pwm_limit
//        if (device.pwm_limit < 0) {
//            device.pwm_limit = 0;
//        } else if (device.pwm_limit > 100) {
//            device.pwm_limit = 100;
//        }
//    }
//}

//void pwm_limit_search() {
//    uint8 index;
//    uint16 max_tension = 25500; 
//    uint16 min_tension = 11500;

//    if (device.tension > max_tension) {
//        device.pwm_limit = 0;
//    } else if (device.tension < min_tension) {
//        device.pwm_limit = 100;
//    } else {
//        index = (uint8)((device.tension - min_tension) / 500);
//        device.pwm_limit = pwm_preload_values[index];
//    }
//}
