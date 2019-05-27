/**
 * \file
 *
 * \brief FreeRTOS configuration
 *
 * Copyright (c) 2012-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
 
/**
 * \mainpage FreeRTOS Real Time Kernel example
 *
 * \section Purpose
 *
 * The FreeRTOS example will help users how to use FreeRTOS in SAM boards.
 * This basic application shows how to create task and get information of
 * created task.
 *
 * \section Requirements
 *
 * This package can be used with SAM boards.
 *
 * \section Description
 *
 * The demonstration program create two task, one is make LED on the board
 * blink at a fixed rate, and another is monitor status of task.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a>
 *    application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>,
 *    depending on your chosen solution.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# LED should start blinking on the board. In the terminal window, the
 *    following text should appear (values depend on the board and chip used):
 *    \code
	-- Freertos Example xxx --
	-- xxxxxx-xx
	-- Compiled: xxx xx xxxx xx:xx:xx --
\endcode
 *
 */

#include <asf.h>
#include <string.h>
#include <stdio.h>
#include "conf_board.h"

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (3)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

// Leds

#define LED_PIO_IDB	   ID_PIOC
#define LED_PIOB        PIOC
#define LED_PINB		   13
#define LED_PIN_MASKB   (1<<LED_PINB)

#define LED_PIO_IDY	   ID_PIOA
#define LED_PIOY        PIOA
#define LED_PINY		   4
#define LED_PIN_MASKY   (1<<LED_PINY)

#define LED_PIO_IDR	   ID_PIOA
#define LED_PIOR        PIOA
#define LED_PINR		   24
#define LED_PIN_MASKR   (1<<LED_PINR)

#define LED_PIO_IDG	   ID_PIOD
#define LED_PIOG        PIOD
#define LED_PING		   26
#define LED_PIN_MASKG   (1<<LED_PING)

#define LED_PIO_IDW	   ID_PIOD
#define LED_PIOW        PIOD
#define LED_PINW		   11
#define LED_PIN_MASKW   (1<<LED_PINW)

typedef struct nota t_nota;
typedef struct notadastruct stru_nota;
struct nota{
	int note;
	int tempo;
};

struct notadastruct{
	int n;
	int led;
	int mask;
};

//MUSICA TESTE
int DO = 1;
int RE = 2;
int MI = 3;
int FA = 4;
int SOL = 5;
int PAUSA = 0;
int UM = 2;
int MEIO = 1;
int N = 29;

stru_nota task_DO = {.n = 1, .led = LED_PIOB, .mask = LED_PIN_MASKB};
int task_RE[] = {2, LED_PIOY, LED_PIN_MASKY};
int task_MI[] = {3, LED_PIOR, LED_PIN_MASKR};
int task_FA[] = {4, LED_PIOG, LED_PIN_MASKG};
int task_SOL[] = {5, LED_PIOW, LED_PIN_MASKW};

const t_nota P1 = {.note = 0,
	 .tempo = 1,};
const t_nota C1 = {.note = 1, 
	.tempo = 1};
const t_nota D1 = {.note = 2, 
	.tempo = 1};
const t_nota E1 ={.note = 3, 
	.tempo = 1};
const t_nota F2 ={.note = 4, 
	.tempo = 2};
const t_nota F1 ={.note = 4, 
	.tempo = 1};
const t_nota D2 ={.note = 2, 
	.tempo = 2};
const t_nota G1 ={.note = 5, 
	.tempo = 1};
const t_nota E2 ={.note = 3, 
	.tempo = 2};
const t_nota fim ={.note = 1000,
.tempo = 2};

t_nota *DOREMIFA[] = {&P1,&C1,&D1,&E1,&F2,&F1,&F1,&P1,&C1,&D1,&C1,&D2,&D1,&D1,&P1,&C1,&G1,&F1,&E2,&E1,&E1,&P1,&C1,&D1,&E1,&F2,&F1,&F1,&fim};
static int beat = 1000;

QueueHandle_t xQueueMus;
SemaphoreHandle_t xSemaphoreNotes;

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}


void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get(pio, PIO_INPUT, mask)){
		pio_clear(pio, mask);
	}
	
	else{
		pio_set(pio,mask);
	}
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */

void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_IDB);
	pmc_enable_periph_clk(LED_PIO_IDY);
	pmc_enable_periph_clk(LED_PIO_IDR);
	pmc_enable_periph_clk(LED_PIO_IDG);
	pmc_enable_periph_clk(LED_PIO_IDW);
	
	pio_set_output(LED_PIOB, LED_PIN_MASKB, estado, 0, 0 );
	pio_set_output(LED_PIOY, LED_PIN_MASKY, estado, 0, 0 );
	pio_set_output(LED_PIOR, LED_PIN_MASKR, estado, 0, 0 );
	pio_set_output(LED_PIOG, LED_PIN_MASKG, estado, 0, 0 );
	pio_set_output(LED_PIOW, LED_PIN_MASKW, estado, 0, 0 );
};

static void task_maestro()
{
	int delay = 0;
	for (;;) {
		for(int i = 0; i < N; i++){
			printf("iiiiiiiiii: %d    \n", i);
			t_nota *atual = DOREMIFA[i];
			//delay += atual->tempo;
			if(xQueueMus != 0 && atual->note != 1000){
				xQueueSend(xQueueMus,atual,( TickType_t ) 0 ) ;
				if (i == N-1){
					xQueueReset( xQueueMus );
				}
			}
			vTaskDelay(100);
		}
	}
}


static void task_led(void *pvParameters)
{
	
	//stru_nota *id ;
	
    //id = ( (stru_nota* ) pvParameters); // problemas para pegar parâmetro que é um struct
	
	xQueueMus = xQueueCreate( N+10, sizeof( t_nota ) );
	t_nota atual;// = {.note = 0, .tempo = 1};
	//printf("nota %d\n",id[0]);
	LED_init(0);
	for (;;) {
		if( xQueueMus != 0 )
		{
			if( xQueuePeek( xQueueMus, ( &atual ), sizeof( t_nota )*10 ) )
			{
				printf("TEMPO ATUAL %d\n",atual.tempo);
	//		// pcRxedMessage now points to the struct AMessage variable posted
	//		// by vATask, but the item still remains on the queue.
				pio_clear(LED_PIOB,LED_PIN_MASKB);
				pio_clear(LED_PIOY,LED_PIN_MASKY);
				pio_clear(LED_PIOR,LED_PIN_MASKR);
				pio_clear(LED_PIOG,LED_PIN_MASKG);
				pio_clear(LED_PIOW,LED_PIN_MASKW);
				vTaskDelay(100);
				if(atual.note == 1){
					printf("ENTROU DO %d\n",atual.note);
					//printf("TESTE nota DO %d\n",id[0]);
					if( xQueueReceive(xQueueMus, ( &atual ), sizeof( t_nota )*10 ) )
					{
	//					// pcRxedMessage now points to the struct AMessage variable posted
	//					// by vATask.
						pio_set(LED_PIOB,LED_PIN_MASKB);
					}
				}
				else if(atual.note == 2){
					printf("ENTROU RE %d\n",atual.note);
					//printf("TESTE nota RE %d\n",id[0]);
					if( xQueueReceive(xQueueMus, ( &atual ), sizeof( t_nota )*10 ) )
					{
						//					// pcRxedMessage now points to the struct AMessage variable posted
						//					// by vATask.
						pio_set(LED_PIOY,LED_PIN_MASKY);
					}
				}
				else if(atual.note == 3){
					printf("ENTROU MI %d\n",atual.note);
					//printf("TESTE nota RE %d\n",id[0]);
					if( xQueueReceive(xQueueMus, ( &atual ), sizeof( t_nota )*10 ) )
					{
						//					// pcRxedMessage now points to the struct AMessage variable posted
						//					// by vATask.
						pio_set(LED_PIOR,LED_PIN_MASKR);
					}
				}
				else if(atual.note == 4){
					printf("ENTROU FA %d\n",atual.note);
					//printf("TESTE nota RE %d\n",id[0]);
					if( xQueueReceive(xQueueMus, ( &atual ), sizeof( t_nota )*10 ) )
					{
						//					// pcRxedMessage now points to the struct AMessage variable posted
						//					// by vATask.
						pio_set(LED_PIOG,LED_PIN_MASKG);
					}
				}
				else if(atual.note == 5){
					printf("ENTROU SOL %d\n",atual.note);
					//printf("TESTE nota RE %d\n",id[0]);
					if( xQueueReceive(xQueueMus, ( &atual ), sizeof( t_nota )*10 ) )
					{
						//					// pcRxedMessage now points to the struct AMessage variable posted
						//					// by vATask.
						pio_set(LED_PIOW,LED_PIN_MASKW);
					}
				}
				else if(atual.note == 0){
					printf("ENTROU pausa %d\n",atual.note);
					//printf("TESTE nota RE %d\n",id[0]);
					if( xQueueReceive(xQueueMus, ( &atual ), sizeof( t_nota )*10 ) )
					{
						//					// pcRxedMessage now points to the struct AMessage variable posted
						//					// by vATask.
					}
				}
			}
		}
	//vTaskDelay(1000);
	printf("TEMPO ATUAL %d\n",atual.tempo);
	vTaskDelay(atual.tempo*600);
	}
}

static void task_pausa(void *pvParameters)
{
	xQueueMus = xQueueCreate( 20, sizeof( int ) );
	t_nota atual;
	//LED_init(0);
	printf("Entrou pausa %d\n",0);
	for (;;) {
		if( xQueueMus != 0 )
		{
			if( xQueuePeek( xQueueMus, ( &atual ), sizeof( t_nota )*10 ) )
			{
				printf("NOTA ATUAL %d\n",atual.note);
				//		// pcRxedMessage now points to the struct AMessage variable posted
				//		// by vATask, but the item still remains on the queue.
				if(atual.note == 0){
					if( xQueueReceive(xQueueMus, ( &atual ), sizeof( t_nota )*10 ) )
					{
						//					// pcRxedMessage now points to the struct AMessage variable posted
						//					// by vATask.
					}
				}
			}
			//
			//		vTaskDelay(atual.tempo*beat);
		}
		vTaskDelay(1000);
	}
}



/**
 * \brief Configure the console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}



/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();


	/* Create task to make led blink */
	if (xTaskCreate(task_led, "LedB", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test ledB task\r\n");
	}
	
	
	if (xTaskCreate(task_maestro, "maestro", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test MAESTRO task\r\n");
	}
	
/*
	if (xTaskCreate(task_pausa, "pausa", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test pausa task\r\n");
	}*/

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
