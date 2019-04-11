

#ifndef COMMAND_PROCESSING_H_INCLUDED
#define COMMAND_PROCESSING_H_INCLUDED
// ----------------------------------------------------------------------------

//=================================================================     includes
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE

//==============================================================================
//                                                          function definitions
//==============================================================================


//void    paramSet           (uint16 param_type);
//void    paramGet           (uint16 param_type);
void    infoPrepare        (unsigned char *info_string);
void    infoSend           (void);
//void    infoGet            (uint16 info_typea);
void    commProcess        (void);
//void    commWrite          (uint8 *packet_data, uint16 packet_lenght);
//uint8   memStore           (int);
//void    sendAcknowledgment (uint8);
//void    memRecall          (void);
//uint8   memRestore         (void);
//uint8   memInit            (void);
uint8   LCRChecksum        (uint8 *data_array, uint8 data_length);

#endif

