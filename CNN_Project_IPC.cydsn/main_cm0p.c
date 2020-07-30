/******************************************************************************
*   File Name: main_cm0p.c
*  Created on: July 28, 2020
*      Author: Carrillo
*
* Description: This example demonstrates how to use the IPC to implement a 
*              message pipe in PSoC 6 MCU. The pipe is used as a method to 
*              send messages between the CPUs. CM0p will send the 32x32 RGB
*              image to CM4, and CM4 will execute the CNN application
*
* Hardware Dependency: PSoC 6 BLE Pioneer kit CY8CKIT-062-BLE
****************************************************************************/
#include "project.h"
#include <stdio.h>
#include "ipc_def.h"
#include "arm_math.h"
#include "arm_nnexamples_cifar10_parameter.h"
#include "arm_nnexamples_cifar10_weights.h"
#include "arm_nnexamples_cifar10_inputs.h"
/****************************************************************************
*            Global Variables
*****************************************************************************/
ipc_msg_t ipcMsgForCM4 = {              /* IPC structure to be sent to CM4  */
    .clientId = IPC_CM0_TO_CM4_CLIENT_ID,
    .userCode = 0,
    .intrMask = CY_SYS_CYPIPE_INTR_MASK,
    .ptrImgBuffer   = NULL
};

ipc_msg_t *ipcMsgFromCM4;
volatile bool rdyToRecvMsg = true;      /* Ready to receive message flag    */

/****************************************************************************
*            Prototype Functions
*****************************************************************************/
void CM0_ReleaseCallback(void);

uint8_t  image_data_M0p[CONV1_IM_CH * CONV1_IM_DIM * CONV1_IM_DIM] = IMG_DATA;

/*******************************************************************************
* Function Name: main()
********************************************************************************
* Summary:
*   Main function of Cortex-M0p. Handle the UART communication. 
*
*******************************************************************************/

int main(void)
{
    uint32_t character;
        
    __enable_irq(); /* Enable global interrupts. */
    
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    
    /* Initialize the UART block */
    UART_Start();
    
    /* Send welcome message to UART */
    Cy_SCB_UART_PutString(UART_HW, "\r\n---- IPC Pipes Code Example (Image) --------\r\n");
    Cy_SCB_UART_PutString(UART_HW, "\r\n---- Press ENTER to send 32x32 RGB image ---\r\n");
    Cy_SCB_UART_PutString(UART_HW, "\r\n--------------------------------------------\n\n\r> ");
    
    
    /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);
    
    for(;;)
    {
        /* Check if ready to send a message */
        if (rdyToRecvMsg)
        {       
            /* Get one character from the RX fifo*/
            character = Cy_SCB_UART_Get(UART_HW);
            
            /* Only process if a character is received */
            if (character != CY_SCB_UART_RX_NO_DATA )
            {
                switch (character)
                {
                    case '\r':
                        /* If ENTER is pressed, process the command */
                        Cy_SCB_UART_Put(UART_HW, '\n');
                        Cy_SCB_UART_Put(UART_HW, '\r');
                        
                        /* Here ptrImgBuffer points to the RGB image 
                        accessed by CM0+, then sent to CM4.
                        If camera is implemented, there should by a task
                        that extracts the image and stores it in memory. Then 
                        the following line will point to the image to be sent
                        to CM4 */
                        ipcMsgForCM4.ptrImgBuffer = image_data_M0p;
                        
                        /* Clear flag */
                        rdyToRecvMsg = false;
                        
                        /* Send the received characters to CM4 to be processed */
                        Cy_IPC_Pipe_SendMessage(CY_IPC_EP_CYPIPE_CM4_ADDR,
                                                CY_IPC_EP_CYPIPE_CM0_ADDR,
                                                (uint32_t *) &ipcMsgForCM4, CM0_ReleaseCallback);
                        break;
                        
                    case '\b':
                        /* Clear the last character */
                        Cy_SCB_UART_Put(UART_HW, '\b');
                        Cy_SCB_UART_Put(UART_HW, ' ');
                        Cy_SCB_UART_Put(UART_HW, '\b');
                        break;
                        
                    default:
                        /* Send the character back to the terminal */
                        Cy_SCB_UART_Put(UART_HW, character);                        
                        break;
                }
            }
        }        
    }
}


/*******************************************************************************
* Function Name: CM0_ReleaseCallback()
********************************************************************************
* Summary:
*   Callback function that is executed when the CM4 receives a message, freeing
*   this core to send another message to CM4.
*
*******************************************************************************/
void CM0_ReleaseCallback(void)
{
    /* Message processed by CM4. Ready to receive a new message */
    ipcMsgForCM4.ptrImgBuffer = NULL;
    rdyToRecvMsg = true;
}

/* [] END OF FILE */


