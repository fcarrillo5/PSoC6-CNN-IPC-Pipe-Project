/******************************************************************************
*   File Name: main_cm4.c
*  Created on: July 28, 2020
*      Author: Carrillo
*
* Description: This project uses the CM4 processor to execute the application
*
* Hardware Dependency: PSoC 6 BLE Pioneer kit CY8CKIT-062-BLE
****************************************************************************/
#include "project.h"
#include <stdio.h>
#include "ipc_def.h"
#include "arm_math.h"
#include "arm_nnexamples_cifar10_parameter.h"
#include "arm_nnexamples_cifar10_weights.h"
#include "arm_nnfunctions.h"
#include "arm_nnexamples_cifar10_inputs.h"
/*******************************************************************************
*            Global variables
*******************************************************************************/
volatile uint32_t SysTickCnt = 0;         /* Used in SysTick couter         */

volatile bool rdyToProcess = false;      /* Ready to process flag           */

ipc_msg_t *ipcMsgFromCM0;                /* IPC structure received from CM0 */

//Variable used to calculate delay for each CNN function
uint32_t cnt_init = 0;
uint32_t cnt_fin = 0; 
uint32_t scale = 0;
uint32_t t_initial = 0;
uint32_t t_final = 0;
uint32_t total_time_nano = 0;
uint32_t sup_time_nano = 0;
char buffer[50];

/*******************************************************************************
* Function Name: SystickIsrHandler
*******************************************************************************/
void SystickIsrHandler(void)
{
    SysTickCnt++;    
}

/****************************************************************************
*            Prototype Functions
*****************************************************************************/
void CM4_MessageCallback(uint32_t *msg);
void calculateDelay(uint32_t cnt_init, uint32_t cnt_fin, uint32_t scale);

// include the input and weights
static q7_t conv1_wt[CONV1_IM_CH * CONV1_KER_DIM * CONV1_KER_DIM * CONV1_OUT_CH] = CONV1_WT;
static q7_t conv1_bias[CONV1_OUT_CH] = CONV1_BIAS;

static q7_t conv2_wt[CONV2_IM_CH * CONV2_KER_DIM * CONV2_KER_DIM * CONV2_OUT_CH] = CONV2_WT;
static q7_t conv2_bias[CONV2_OUT_CH] = CONV2_BIAS;

static q7_t conv3_wt[CONV3_IM_CH * CONV3_KER_DIM * CONV3_KER_DIM * CONV3_OUT_CH] = CONV3_WT;
static q7_t conv3_bias[CONV3_OUT_CH] = CONV3_BIAS;

static q7_t ip1_wt[IP1_DIM * IP1_OUT] = IP1_WT;
static q7_t ip1_bias[IP1_OUT] = IP1_BIAS;

/* Here the image_data should be the raw uint8 type RGB image in [RGB, RGB, RGB ... RGB] format */
uint8_t   image_data[CONV1_IM_CH * CONV1_IM_DIM * CONV1_IM_DIM];
q7_t      output_data[IP1_OUT];

//vector buffer: max(im2col buffer,average pool buffer, fully connected buffer)
q7_t      col_buffer[2 * 5 * 5 * 32 * 2];

q7_t      scratch_buffer[32 * 32 * 10 * 4];

int main(void)
{
    
    __enable_irq(); /* Enable global interrupts. */
    
        /* Initialize SysTick timer */
    // In order to get 1ms delay for each SysTick interrupt, we need 
    // to set the reload/interval to 7999. See formula below
    // (7999) reload = 1 ms delay each interrupt * Clock Frequency (IMO = 8Mhz) - 1
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_IMO, 0x00FFFFFF);  //Interval = 16,777,216 - 1
    // 16,777,215 / 8 MHz = 2.097 sec or 2097 msec for each systick interrupt
    
    uint32_t i;
    for(i = 0u; i < CY_SYS_SYST_NUM_OF_CALLBACKS; ++i)
    {
        if(Cy_SysTick_GetCallback(i) == NULL)
        {   
            Cy_SysTick_SetCallback(i, SystickIsrHandler);
            break;
        }
    } 
    

    /* Register the Message Callback */
    Cy_IPC_Pipe_RegisterCallback(CY_IPC_EP_CYPIPE_ADDR,
                                 CM4_MessageCallback,
                                 IPC_CM0_TO_CM4_CLIENT_ID);    

    for(;;)
    {
        /* Check if ready to process message */
        if (rdyToProcess)
        {
            /* Clear the flag */
            rdyToProcess = false;
            
            /* Process only if pointer to image is not NULL */
            if (ipcMsgFromCM0->ptrImgBuffer[0] != 0)
            {
    
                /* start the execution */
                q7_t     *img_buffer1 = scratch_buffer;
                q7_t     *img_buffer2 = img_buffer1 + 32 * 32 * 32;   
    
                Cy_SCB_UART_PutString(UART_HW, "Input Pre-processing\r\n");
                /* input pre-processing */
                int mean_data[3] = INPUT_MEAN_SHIFT;
                unsigned int scale_data[3] = INPUT_RIGHT_SHIFT;
                SysTickCnt = 0;
                cnt_init = Cy_SysTick_GetValue();
                for (int i=0;i<32*32*3; i+=3) {
                    img_buffer2[i] =   (q7_t)__SSAT( ((((int)image_data[i]   - mean_data[0])<<7) + (0x1<<(scale_data[0]-1)))
                                             >> scale_data[0], 8);
                    img_buffer2[i+1] = (q7_t)__SSAT( ((((int)image_data[i+1] - mean_data[1])<<7) + (0x1<<(scale_data[1]-1)))
                                             >> scale_data[1], 8);
                    img_buffer2[i+2] = (q7_t)__SSAT( ((((int)image_data[i+2] - mean_data[2])<<7) + (0x1<<(scale_data[2]-1)))
                                             >> scale_data[2], 8);
                }    
                cnt_fin = Cy_SysTick_GetValue();
                scale = SysTickCnt;
                calculateDelay(cnt_init, cnt_fin, scale);
                Cy_SCB_UART_PutString(UART_HW,"Input Pre-processing completed\r\n\n\n");
                
                //*************************************************************************
    
                Cy_SCB_UART_PutString(UART_HW,"Performing first convolution RGB\r\n");
                Cy_SCB_UART_PutString(UART_HW,"arm_convolve_HWC_q7_RGB()\r\n");
                // conv1 img_buffer2 -> img_buffer1
                SysTickCnt = 0;
                cnt_init = Cy_SysTick_GetValue();
                arm_convolve_HWC_q7_RGB(img_buffer2, CONV1_IM_DIM, CONV1_IM_CH, conv1_wt, CONV1_OUT_CH, CONV1_KER_DIM, CONV1_PADDING,
                                        CONV1_STRIDE, conv1_bias, CONV1_BIAS_LSHIFT, CONV1_OUT_RSHIFT, img_buffer1, CONV1_OUT_DIM,
                                        (q15_t *) col_buffer, NULL); 
                cnt_fin = Cy_SysTick_GetValue();
                scale = SysTickCnt;
                calculateDelay(cnt_init, cnt_fin, scale);
                Cy_SCB_UART_PutString(UART_HW,"Convolution RGB completed\r\n\n\n");
                
                //*************************************************************************
    
                SysTickCnt = 0;
                cnt_init = Cy_SysTick_GetValue();
                Cy_SCB_UART_PutString(UART_HW,"Performing first arm_relu_q7\r\n");
                arm_relu_q7(img_buffer1, CONV1_OUT_DIM * CONV1_OUT_DIM * CONV1_OUT_CH);
                cnt_fin = Cy_SysTick_GetValue();
                scale = SysTickCnt;
                calculateDelay(cnt_init, cnt_fin, scale);
                Cy_SCB_UART_PutString(UART_HW,"First arm_relu_q7 completed\r\n\n\n");
                
                //*************************************************************************
                
                Cy_SCB_UART_PutString(UART_HW,"Performing first arm_maxpool_q7_HWC\r\n");
                // pool1 img_buffer1 -> img_buffer2
                SysTickCnt = 0;
                cnt_init = Cy_SysTick_GetValue();
                arm_maxpool_q7_HWC(img_buffer1, CONV1_OUT_DIM, CONV1_OUT_CH, POOL1_KER_DIM,
                                   POOL1_PADDING, POOL1_STRIDE, POOL1_OUT_DIM, NULL, img_buffer2);
                cnt_fin = Cy_SysTick_GetValue();
                scale = SysTickCnt;
                calculateDelay(cnt_init, cnt_fin, scale);
                Cy_SCB_UART_PutString(UART_HW,"First arm_maxpool_q7_HWC completed\r\n\n\n");
                
                //*************************************************************************
                
                Cy_SCB_UART_PutString(UART_HW,"Performing second convolution\r\n");
                Cy_SCB_UART_PutString(UART_HW,"arm_convolve_HWC_q7_fast()\r\n");
                // conv2 img_buffer2 -> img_buffer1  
                SysTickCnt = 0;
                cnt_init = Cy_SysTick_GetValue();
                arm_convolve_HWC_q7_fast(img_buffer2, CONV2_IM_DIM, CONV2_IM_CH, conv2_wt, CONV2_OUT_CH, CONV2_KER_DIM,
                                         CONV2_PADDING, CONV2_STRIDE, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, img_buffer1,
                                         CONV2_OUT_DIM, (q15_t *) col_buffer, NULL);
                cnt_fin = Cy_SysTick_GetValue();
                scale = SysTickCnt;
                calculateDelay(cnt_init, cnt_fin, scale);
                Cy_SCB_UART_PutString(UART_HW,"Second convolution completed\r\n\n\n");

                //*************************************************************************
                
                Cy_SCB_UART_PutString(UART_HW,"Performing second arm_relu_q7\r\n");
                SysTickCnt = 0;
                cnt_init = Cy_SysTick_GetValue();
                arm_relu_q7(img_buffer1, CONV2_OUT_DIM * CONV2_OUT_DIM * CONV2_OUT_CH);
                cnt_fin = Cy_SysTick_GetValue();
                scale = SysTickCnt;
                calculateDelay(cnt_init, cnt_fin, scale);
                Cy_SCB_UART_PutString(UART_HW,"Second arm_relu_q7 completed\r\n\n\n");

                //*************************************************************************
                
                Cy_SCB_UART_PutString(UART_HW,"Performing second arm_maxpool_q7_HWC\r\n");
                // pool2 img_buffer1 -> img_buffer2
                SysTickCnt = 0;
                cnt_init = Cy_SysTick_GetValue();
                arm_maxpool_q7_HWC(img_buffer1, CONV2_OUT_DIM, CONV2_OUT_CH, POOL2_KER_DIM,
                                   POOL2_PADDING, POOL2_STRIDE, POOL2_OUT_DIM, col_buffer, img_buffer2);
                cnt_fin = Cy_SysTick_GetValue();
                scale = SysTickCnt;
                calculateDelay(cnt_init, cnt_fin, scale);
                Cy_SCB_UART_PutString(UART_HW,"Second arm_maxpool_q7_HWC completed\r\n\n\n");
                
                //*************************************************************************

                Cy_SCB_UART_PutString(UART_HW,"Performing third convolution\r\n");
                printf("arm_convolve_HWC_q7_fast()\r\n");
                // conv3 img_buffer2 -> img_buffer1
                SysTickCnt = 0;
                cnt_init = Cy_SysTick_GetValue();
                arm_convolve_HWC_q7_fast(img_buffer2, CONV3_IM_DIM, CONV3_IM_CH, conv3_wt, CONV3_OUT_CH, CONV3_KER_DIM,
                                         CONV3_PADDING, CONV3_STRIDE, conv3_bias, CONV3_BIAS_LSHIFT, CONV3_OUT_RSHIFT, img_buffer1,
                                         CONV3_OUT_DIM, (q15_t *) col_buffer, NULL);
                cnt_fin = Cy_SysTick_GetValue();
                scale = SysTickCnt;
                calculateDelay(cnt_init, cnt_fin, scale);
                Cy_SCB_UART_PutString(UART_HW,"Third convolution completed\r\n\n\n");

                //*************************************************************************
                
                Cy_SCB_UART_PutString(UART_HW,"\nPerforming third arm_relu_q7\r\n");
                SysTickCnt = 0;
                cnt_init = Cy_SysTick_GetValue();
                arm_relu_q7(img_buffer1, CONV3_OUT_DIM * CONV3_OUT_DIM * CONV3_OUT_CH);
                cnt_fin = Cy_SysTick_GetValue();
                scale = SysTickCnt;
                calculateDelay(cnt_init, cnt_fin, scale);
                Cy_SCB_UART_PutString(UART_HW,"Third arm_relu_q7 completed\r\n\n\n");

                //*************************************************************************
                
                Cy_SCB_UART_PutString(UART_HW,"Performing third arm_maxpool_q7_HWC\r\n");
                // pool3 img_buffer-> img_buffer2
                SysTickCnt = 0;
                cnt_init = Cy_SysTick_GetValue();
                arm_maxpool_q7_HWC(img_buffer1, CONV3_OUT_DIM, CONV3_OUT_CH, POOL3_KER_DIM,
                                   POOL3_PADDING, POOL3_STRIDE, POOL3_OUT_DIM, col_buffer, img_buffer2);
                cnt_fin = Cy_SysTick_GetValue();
                scale = SysTickCnt;
                calculateDelay(cnt_init, cnt_fin, scale);
                Cy_SCB_UART_PutString(UART_HW,"Third arm_maxpool_q7_HWC completed\r\n\n\n");
                
                //*************************************************************************
                
                Cy_SCB_UART_PutString(UART_HW,"\nPerforming arm_fully_connected_q7_opt\r\n");
                SysTickCnt = 0;
                cnt_init = Cy_SysTick_GetValue();
                arm_fully_connected_q7_opt(img_buffer2, ip1_wt, IP1_DIM, IP1_OUT, IP1_BIAS_LSHIFT, IP1_OUT_RSHIFT, ip1_bias,
                                           output_data, (q15_t *) img_buffer1);
                cnt_fin = Cy_SysTick_GetValue();
                scale = SysTickCnt;
                calculateDelay(cnt_init, cnt_fin, scale);
                Cy_SCB_UART_PutString(UART_HW,"arm_fully_connected_q7_opt completed\r\n\n\n");

                //*************************************************************************
                
                Cy_SCB_UART_PutString(UART_HW,"\nPerforming arm_softmax_q7\r\n");
                SysTickCnt = 0;
                cnt_init = Cy_SysTick_GetValue();
                arm_softmax_q7(output_data, 10, output_data);
                cnt_fin = Cy_SysTick_GetValue();
                scale = SysTickCnt;
                calculateDelay(cnt_init, cnt_fin, scale);
                Cy_SCB_UART_PutString(UART_HW,"arm_softmax_q7 completed\r\n\n\n");

                for (int i = 0; i < 10; i++)
                {
                    printf("%d: %d\r\n", i, output_data[i]);
                }
                
                Cy_SCB_UART_PutString(UART_HW, "\r\n\n> ");
            }
            else
            {
                /* If no image received */
                CyDelay(500);
                Cy_SCB_UART_PutString(UART_HW, "\r\n\n\nERROR!\r\n\n");
                Cy_SCB_UART_PutString(UART_HW, "> ");
            }
            
        }
    }
}

/****************************************************************************
* Function Name: CM4_MessageCallback()
*****************************************************************************
* Summary:
*   Callback function that is executed when a message is received from 
*   CM0+. Copy the message to a local array, which is processed in the
*   main loop.
*
* Parameters:
*   msg: IPC message received
*
****************************************************************************/
void CM4_MessageCallback(uint32_t *msg)
{    
    if (msg != NULL)
    {
        /* Cast the message received to the IPC structure */
        ipcMsgFromCM0 = (ipc_msg_t *) msg;
        
        /* Copy image data */
        for (int i=0; i < 32*32*3; i++){
            image_data[i] = ipcMsgFromCM0->ptrImgBuffer[i];
        }
                
        /* Set flag to the main loop to process string */
        rdyToProcess = true;
    }
}

/*******************************************************************************
* Function Name: calculateDelay
*******************************************************************************/
void calculateDelay(uint32_t cnt_init, uint32_t cnt_fin, uint32_t scale){
    
    t_initial = cnt_init * 125;    
    
    if(scale == 0)
    {
        t_final = cnt_fin * 125;
        total_time_nano = t_initial - t_final; // units in nano sec
        printf("Total time in nano seconds = %u \r\n", total_time_nano);
    }
    else
    {
        t_final = (16777215 - cnt_fin) * 125;
        sup_time_nano = t_initial + t_final; // need to add (2.097 sec * (scale - 1))
        printf("Total time in nano seconds: 2.097[sec] * %u + %u[nano seconds]\r\n",
              (scale - 1u), sup_time_nano);
    }      
}
/* [] END OF FILE */