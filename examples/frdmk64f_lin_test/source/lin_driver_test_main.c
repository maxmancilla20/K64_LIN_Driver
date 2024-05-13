/*
 * lin_driver_test_main.c
 * Created on: Sep 15, 2018
 *     Author: Nico
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_common.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"
#include "fsl_port.h"

#include "pin_mux.h"
#include "clock_config.h"
#include <lin1d3_driver.h>
#include <SwitchesLedsInit.h>
#include "FreeRTOSConfig.h"

#include "SwitchesLedsInit.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define MASTER
#define SLAVE_C

/* UART instance and clock */
#define MASTER_UART UART3
#define MASTER_UART_CLKSRC UART3_CLK_SRC
#define MASTER_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define MASTER_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define LOCAL_SLAVE_UART UART3
#define LOCAL_SLAVE_UART_CLKSRC UART3_CLK_SRC
#define LOCAL_SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define LOCAL_SLAVE_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define SLAVE_UART UART4
#define SLAVE_UART_CLKSRC UART4_CLK_SRC
#define SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART4_CLK_SRC)
#define SLAVE_UART_RX_TX_IRQn UART4_RX_TX_IRQn

/* Task priorities. */
#define init_task_PRIORITY (configMAX_PRIORITIES - 2)
#define test_task_heap_size_d	(192)

#define app_message_id_1_d (0x01<<2|message_size_2_bytes_d)
#define app_message_id_2_d (0x02<<2|message_size_4_bytes_d)
#define app_message_id_3_d (0x03<<2|message_size_8_bytes_d)
#define app_message_id_4_d (0x04<<2|message_size_2_bytes_d)
#define app_message_id_5_d (0x05<<2|message_size_4_bytes_d)
#define app_message_id_6_d (0x06<<2|message_size_8_bytes_d)

#define LEDSEND 0xFF
/* GPIO pin configuration. */
#define BOARD_LED_GPIO       BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN   BOARD_LED_RED_GPIO_PIN
#define BOARD_SW_GPIO        BOARD_SW3_GPIO
#define BOARD_SW_GPIO_PIN    BOARD_SW3_GPIO_PIN
#define BOARD_SW_PORT        BOARD_SW3_PORT
#define BOARD_SW_IRQ         BOARD_SW3_IRQ
#define BOARD_SW_IRQ_HANDLER BOARD_SW3_IRQ_HANDLER


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void test_task(void *pvParameters);

/*static void	message_4_callback_local_slave(void* message);
static void	message_5_callback_local_slave(void* message);
static void	message_6_callback_local_slave(void* message);
static void	message_1_callback_local_slave(void* message);

static void	message_1_callback_slave(void* message);
static void	message_2_callback_slave(void* message);
static void	message_3_callback_slave(void* message);*/

#if defined(SLAVE_C)
static void	message_1_callback_slave(void* message);
#endif

//void IDSimulation(void);
void BOARD_SW2_IRQ_HANDLER(void);
void BOARD_SW3_IRQ_HANDLER(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool button1_pressed;
volatile bool button2_pressed;

int error;
lin1d3_nodeConfig_t node_config;
#if defined(MASTER)
	lin1d3_handle_t* master_handle;
#endif
#if defined(SLAVE_C)
	lin1d3_handle_t* slave_handle;
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
void BOARD_SW2_IRQ_HANDLER(void)
{
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW2_GPIO, 1U << BOARD_SW2_GPIO_PIN);

#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
	button1_pressed = 1;
}

void BOARD_SW3_IRQ_HANDLER(void)
{
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW3_GPIO, 1U << BOARD_SW3_GPIO_PIN);

#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
	button2_pressed = 1;
}

int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    NVIC_SetPriority(MASTER_UART_RX_TX_IRQn, 5);
    NVIC_SetPriority(SLAVE_UART_RX_TX_IRQn, 5);
	InitSwLed();

	#if defined(MASTER)
		PRINTF(" *** LIN MASTER ***\r\n");
	#endif
	#if defined(SLAVE_C)
		PRINTF(" *** LIN SLAVE C ***\r\n");
	#endif

    if (xTaskCreate(test_task, "test_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Init Task creation failed!.\r\n");
        while (1)
            ;
    }
    PRINTF(" *** LIN driver demo ***\r\n");
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for loopback.
 */
static void test_task(void *pvParameters)
{

#if defined(MASTER)
	/* Set Master Config */
	node_config.type = lin1d3_master_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = MASTER_UART;
	node_config.srcclk = MASTER_UART_CLK_FREQ;
	node_config.irq = MASTER_UART_RX_TX_IRQn;
	node_config.skip_uart_init = 0;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));

	/* Init Master node */
	master_handle = lin1d3_InitNode(node_config);
#endif
#if defined(SLAVE_C)
		/* Set Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = LOCAL_SLAVE_UART;
	node_config.srcclk = LOCAL_SLAVE_UART_CLK_FREQ;
	node_config.irq = LOCAL_SLAVE_UART_RX_TX_IRQn;
	node_config.skip_uart_init = 0;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = 0;
	node_config.messageTable[0].handler = message_1_callback_slave;
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);
#endif

	if(
#if defined(MASTER)
		(NULL == master_handle)
#endif
#if defined(SLAVE_C)
		 (NULL == slave_handle)
#endif
	   )
	   {
		PRINTF(" Init failed!! \r\n");
		error = kStatus_Fail;
	}
	else {
		error = kStatus_Success;
	}


	while (kStatus_Success == error)
    {
	#if defined(MASTER)
		/*Master task, send the messages ID for LIN*/
		lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
		vTaskDelay(100);
    		
    	lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
    	vTaskDelay(100);

    	lin1d3_masterSendMessage(master_handle, app_message_id_3_d);
		vTaskDelay(100);
    	
	#endif
    	vTaskDelay(100);
    }

    vTaskSuspend(NULL);
}

#if defined(SLAVE_C)
static void	message_1_callback_slave(void* message)
{
	/*SLAVE C SHORT APPLICATION*/
	uint8_t* MessageData = (uint8_t*)message;
	PRINTF("Slave C requested \r\n");

	if(button1_pressed == 1 && button2_pressed == 1 ){
		button1_pressed = 0;
		button2_pressed = 0;	
		PRINTF("LED BLUE \r\n");
		GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_GPIO_PIN);
	}
	else if(button1_pressed == 1 && button2_pressed == 0 ){	
		button1_pressed = 0;
		button2_pressed = 0;
		PRINTF("LED RED \r\n");
		GPIO_PortClear(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_GPIO_PIN);
	}
	else if(button1_pressed == 0 && button2_pressed == 1 ){
		PRINTF("LED PURPLE \r\n");
		button1_pressed = 0;
		button2_pressed = 0;
		GPIO_PortClear(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
		GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_GPIO_PIN);

		xQueueSendFromISR(slave_handle, 0xFF, 0);/*Send something*/
	}
	else if(button1_pressed == 0 && button2_pressed == 0 ){
		button1_pressed = 0;
		button2_pressed = 0;
		PRINTF("OFF \r\n");
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u <<BOARD_LED_GREEN_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
	}
	else{
		PRINTF("WRONG COMMAND\r\n");
	}
	PRINTF("RECEIVED MESSAGE FROM MASTER %d,%d\r\n", MessageData[0], MessageData[1]);
}
#endif
