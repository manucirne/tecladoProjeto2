#include "asf.h"
#include "main.h"
#include <string.h>
#include "bsp/include/nm_bsp.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include <stdio.h>
#include "conf_board.h"

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 weather client example --"STRING_EOL	\
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/** IP address of host. */
uint32_t gu32HostIp = 0;

/** TCP client socket handlers. */
static SOCKET tcp_client_socket = -1;

/** Receive buffer definition. */
static uint8_t gau8ReceivedBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};

/** Wi-Fi status variable. */
static bool gbConnectedWifi = false;

/** Get host IP status variable. */
/** Wi-Fi connection state */
static uint8_t wifi_connected;


/** Instance of HTTP client module. */
static bool gbHostIpByName = false;

/** TCP Connection status variable. */
static bool gbTcpConnection = false;

/** Server host name. */
static char server_host_name[] = MAIN_SERVER_NAME;

/** Server host ip. */
volatile char server_host_ip[15];


#define TASK_WIFI_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_WIFI_STACK_PRIORITY        (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

#define CHAR_SIZE 100
#define LINE_SIZE 21
#define MAX_NOTES 100
int music_matrix[MAX_NOTES][LINE_SIZE];
QueueHandle_t sdQueue;
SemaphoreHandle_t xSemaphoreMusic;
QueueHandle_t xQueueMus;
int N = 0;

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (3)

#include "leds.h"

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


/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}


void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_IDB);
	pmc_enable_periph_clk(LED_PIO_IDY);
	pmc_enable_periph_clk(LED_PIO_IDR);
	pmc_enable_periph_clk(LED_PIO_IDG);
	pmc_enable_periph_clk(LED_PIO_IDW);
	pmc_enable_periph_clk(LED_PIO_IDDO2);
	pmc_enable_periph_clk(LED_PIO_IDLA);
	pmc_enable_periph_clk(LED_PIO_IDSI);
	
	pio_set_output(LED_PIOB, LED_PIN_MASKB, estado, 0, 0 );
	pio_set_output(LED_PIOY, LED_PIN_MASKY, estado, 0, 0 );
	pio_set_output(LED_PIOR, LED_PIN_MASKR, estado, 0, 0 );
	pio_set_output(LED_PIOG, LED_PIN_MASKG, estado, 0, 0 );
	pio_set_output(LED_PIOW, LED_PIN_MASKW, estado, 0, 0 );
	pio_set_output(LED_PIOLA, LED_PIN_MASKLA, estado, 0, 0 );
	pio_set_output(LED_PIOSI, LED_PIN_MASKSI, estado, 0, 0 );
	pio_set_output(LED_PIODO2, LED_PIN_MASKDO2, estado, 0, 0 );
};

static void task_maestro()
{
	int delay = 0;
	for (;;) {
		if( xSemaphoreTake(xSemaphoreMusic, ( TickType_t ) 500) == pdTRUE ){
			for(int i = 0; i < N; i++){
				int *atual = &music_matrix[i];
				if(xQueueMus != 0){
					xQueueSend(xQueueMus,atual,( TickType_t ) 0 ) ;
				}
				vTaskDelay(300);
			}
			N = 0;
		}
	}
}

void TOCA_NOTA(int estado, int p_pio, const uint32_t ul_mask ){
	if(estado){
		pio_set(p_pio,ul_mask);
	}
	else{
		pio_clear(p_pio,ul_mask);
	}
}


static void task_led(void *pvParameters)
{
	
	vTaskDelay(100);
	int atuais[24];
	LED_init(0);
	for (;;) {
		if( xQueueMus != 0 )
		{
			if( xQueueReceive(xQueueMus, ( &atuais ), sizeof( int )*24 ) )
			{
				TOCA_NOTA(atuais[0], LED_PIOB,LED_PIN_MASKB);
				TOCA_NOTA(atuais[1], LED_PIOY,LED_PIN_MASKY);
				TOCA_NOTA(atuais[2], LED_PIOR,LED_PIN_MASKR);
				TOCA_NOTA(atuais[3], LED_PIOG,LED_PIN_MASKG);
				TOCA_NOTA(atuais[4], LED_PIOW,LED_PIN_MASKW);
				TOCA_NOTA(atuais[5], LED_PIOLA,LED_PIN_MASKLA);
				TOCA_NOTA(atuais[6], LED_PIOSI,LED_PIN_MASKSI);
				TOCA_NOTA(atuais[7], LED_PIODO2,LED_PIN_MASKDO2);
				
			}
			
		}
		vTaskDelay(300);
	}
}

/* 
 * Check whether "cp" is a valid ascii representation
 * of an Internet address and convert to a binary address.
 * Returns 1 if the address is valid, 0 if not.
 * This replaces inet_addr, the return value from which
 * cannot distinguish between failure and a local broadcast address.
 */
 /* http://www.cs.cmu.edu/afs/cs/academic/class/15213-f00/unpv12e/libfree/inet_aton.c */
int inet_aton(const char *cp, in_addr *ap)
{
  int dots = 0;
  register u_long acc = 0, addr = 0;

  do {
	  register char cc = *cp;

	  switch (cc) {
	    case '0':
	    case '1':
	    case '2':
	    case '3':
	    case '4':
	    case '5':
	    case '6':
	    case '7':
	    case '8':
	    case '9':
	        acc = acc * 10 + (cc - '0');
	        break;

	    case '.':
	        if (++dots > 3) {
		    return 0;
	        }
	        /* Fall through */

	    case '\0':
	        if (acc > 255) {
		    return 0;
	        }
	        addr = addr << 8 | acc;
	        acc = 0;
	        break;

	    default:
	        return 0;
    }
  } while (*cp++) ;

  /* Normalize the address */
  if (dots < 3) {
	  addr <<= 8 * (3 - dots) ;
  }

  /* Store it if requested */
  if (ap) {
	  ap->s_addr = _htonl(addr);
  }

  return 1;    
}


/**
 * \brief Callback function of IP address.
 *
 * \param[in] hostName Domain name.
 * \param[in] hostIp Server IP.
 *
 * \return None.
 */
static void resolve_cb(uint8_t *hostName, uint32_t hostIp)
{
	gu32HostIp = hostIp;
	gbHostIpByName = true;
	printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", hostName,
			(int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
			(int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
			
	sprintf(server_host_ip,"%d.%d.%d.%d",(int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),(int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
}

/**
 * \brief Callback function of TCP client socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg Type of Socket notification
 * \param[in] pvMsg A structure contains notification informations.
 *
 * \return None.
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
  
	/* Check for socket event on TCP socket. */
	if (sock == tcp_client_socket) {
    
		switch (u8Msg) {
		case SOCKET_MSG_CONNECT:
		{
      printf("socket_msg_connect\n"); 
			if (gbTcpConnection) {
				memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
				sprintf((char *)gau8ReceivedBuffer, "%s", MAIN_PREFIX_BUFFER);

				tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
				if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR) {
          printf("send \n");
					send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);

					memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
					recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
				} else {
					printf("socket_cb: connect error!\r\n");
					gbTcpConnection = false;
					close(tcp_client_socket);
					tcp_client_socket = -1;
				}
			}
		}
		break;
    


		case SOCKET_MSG_RECV:
		{
			char *pcIndxPtr;
			char *pcEndPtr;

			tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
			if (pstrRecv && pstrRecv->s16BufferSize > 0) {
        		char *result = strstr(pstrRecv->pu8Buffer, "musicName:");
				
				if(result != NULL){
					char music_server[CHAR_SIZE];
					char music_name[CHAR_SIZE];
					memset(music_server,0,CHAR_SIZE);
					memset(music_name,0,CHAR_SIZE);
				
					int i = 0;
					result += 10;
					while (*result != '"'){
						music_server[i] = *result;
						i++;
						result++;
					}
					sprintf(music_name,"0:%s",music_server);
					
					xQueueSend( sdQueue, &music_name, NULL);
				}
				
				memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
				recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
			} else {
				printf("socket_cb: recv error!\r\n");
				close(tcp_client_socket);
				tcp_client_socket = -1;
			}
		}
		break;

		default:
			break;
		}
	}
}

static void set_dev_name_to_mac(uint8_t *name, uint8_t *mac_addr)
{
	/* Name must be in the format WINC1500_00:00 */
	uint16 len;

	len = m2m_strlen(name);
	if (len >= 5) {
		name[len - 1] = MAIN_HEX2ASCII((mac_addr[5] >> 0) & 0x0f);
		name[len - 2] = MAIN_HEX2ASCII((mac_addr[5] >> 4) & 0x0f);
		name[len - 4] = MAIN_HEX2ASCII((mac_addr[4] >> 0) & 0x0f);
		name[len - 5] = MAIN_HEX2ASCII((mac_addr[4] >> 4) & 0x0f);
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType Type of Wi-Fi notification.
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters.
 *
 * \return None.
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
			gbConnectedWifi = false;
 			wifi_connected = 0;
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		wifi_connected = M2M_WIFI_CONNECTED;
		
    /* Obtain the IP Address by network name */
		gethostbyname((uint8_t *)server_host_name);
		break;
	}

	default:
	{
		break;
	}
	}
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		printf("--- task ## %u", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(1000);
	}
}



static void task_wifi(void *pvParameters) {
	tstrWifiInitParam param;
	int8_t ret;
	uint8_t mac_addr[6];
	uint8_t u8IsMacAddrValid;
	struct sockaddr_in addr_in;
	
	/* Initialize the BSP. */
	nm_bsp_init();
	
	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}
	
	/* Initialize socket module. */
	socketInit();

	/* Register socket callback function. */
	registerSocketCallback(socket_cb, resolve_cb);
	
	m2m_wifi_set_mac_address(gau8MacAddr);

	/* Connect to router. */
	printf("main: connecting to WiFi AP %s...\r\n", (char *)MAIN_WLAN_SSID);
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	addr_in.sin_family = AF_INET;
	addr_in.sin_port = _htons(MAIN_SERVER_PORT);
	
  while(1){
	  m2m_wifi_handle_events(NULL);

	  if (wifi_connected == M2M_WIFI_CONNECTED) {
		  /* Open client socket. */
		  if (tcp_client_socket < 0) {
			  printf("socket init \n");
			  if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
				  printf("main: failed to create TCP client socket error!\r\n");
				  continue;
			  }

			  /* Connect server */
			  printf("socket connecting\n");
			  inet_aton(server_host_ip, &addr_in.sin_addr);
			  
			  if (connect(tcp_client_socket, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR) {
				  close(tcp_client_socket);
				  tcp_client_socket = -1;
				  printf("error\n");
				  }else{
				  gbTcpConnection = true;
			  }
		  }
	  }
	  }
}

uint read_sdcard(char music_name[]){
	Ctrl_status status;
	FRESULT res;
	FATFS fs;
	FIL file_object;
  
    printf("Please plug an SD, MMC or SDIO card in slot.\n\r");
	
    /* Wait card present and ready */
    do {
      status = sd_mmc_test_unit_ready(0);
      if (CTRL_FAIL == status) {
        printf("Card install FAIL\n\r");
        printf("Please unplug and re-plug the card.\n\r");
        while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
          vTaskDelay(100);
        }
      }
      vTaskDelay(100);
    } while (CTRL_GOOD != status);

    printf("Mount disk (f_mount)...\r\n");
    memset(&fs, 0, sizeof(FATFS));
    res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
    if (FR_INVALID_DRIVE == res) {
      printf("[FAIL] res %d\r\n", res);
      return 1;
    }

    music_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
    res = f_open(&file_object,	(char const *)music_name,	FA_READ);
    if (res != FR_OK) {
      printf("[FAIL] res %d\r\n", res);
      return 1;
    }
	
	UINT bytes_to_read=sizeof(char)*1;
	char infos[bytes_to_read];
	UINT bytes_read=0;
	int i=0;
	int j=0;
	
	do {
		f_read(&file_object,&infos,bytes_to_read,&bytes_read);
		if (strcmp(infos,'1') || strcmp(infos,'0')){
			music_matrix[i][j] = atoi(infos);
			if(j>=LINE_SIZE-1){
				i++;
				j=0;
				N++;
			}
			else{				
				j++;
			}
		}
	} while (bytes_read >= bytes_to_read);
    
    f_close(&file_object);
    return 0;
}


static void task_sdcard(void *pvParameters){
     /* Initialize SD MMC stack */
     printf("\x0C\n\r-- SD/MMC/SDIO Card Example on FatFs --\n\r");
     printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
     
     sd_mmc_init();

	char music_name[CHAR_SIZE];
         
	while(1){
		if (xQueueReceive( sdQueue, &music_name, ( TickType_t )  10 / portTICK_PERIOD_MS )){	
			read_sdcard(music_name);
			xSemaphoreGive(xSemaphoreMusic);
		}
		vTaskDelay(1000);
	}  
}



/**
 * \brief Main application function.
 *
 * Initialize system, UART console, network then start weather client.
 *
 * \return Program return value.
 */
int main(void)
{
	/* Initialize the board. */
	sysclk_init();
	board_init();

	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);

	sdQueue = xQueueCreate( 1, sizeof( char ) * CHAR_SIZE );
	xSemaphoreMusic = xSemaphoreCreateBinary();
	xQueueMus = xQueueCreate( 50, sizeof( int )*24 );
  
	if (xTaskCreate(task_wifi, "Wifi", TASK_WIFI_STACK_SIZE, NULL,
	TASK_WIFI_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Wifi task\r\n");
	}

	if (xTaskCreate(task_sdcard, "sd", TASK_WIFI_STACK_SIZE, NULL,
	TASK_WIFI_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Wifi task\r\n");
	}
	
	/*if (xTaskCreate(task_led, "TOCA", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test TASK TOCA task\r\n");
	}	
	
	if (xTaskCreate(task_maestro, "maestro", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test MAESTRO task\r\n");
	}*/


	vTaskStartScheduler();
	
	while(1) {};
	return 0;

}
