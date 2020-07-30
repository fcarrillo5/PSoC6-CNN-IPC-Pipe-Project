/*****************************************************************************
* File Name		: ipc_def.h
* Version		: 1.0 
*
* Description:
*  Auxiliary header with constants and structures for the IPC.
*
*******************************************************************************/
#ifndef IPC_DEF_H
#define IPC_DEF_H	
    
    #include <stdint.h>
    #include "arm_math.h"
        
    //#define IPC_BUFFER_SIZE                 256
    #define IPC_CM0_TO_CM4_CLIENT_ID        0
    #define IPC_CM4_TO_CM0_CLIENT_ID        1
      
    typedef struct __attribute__((packed, aligned(4)))
    {
        uint8_t     clientId;
        uint8_t     userCode;
        uint16_t    intrMask;
        uint8_t     *ptrImgBuffer;
    } ipc_msg_t ;
    
#endif /* IPC_DEF_H */

/* [] END OF FILE */
