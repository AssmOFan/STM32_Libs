#ifndef DT_UART_
#define DT_UART_

/* Includes ---------------------------------- */
#include <stdbool.h>
#include "stm32f10x.h"
#include "dt_common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Macros ------------------------------------ */
// uncomment define to enable UART, comment to disable UART
#define	UART1_BAUDRATE	115200
//#define	UART2_BAUDRATE	115200
//#define	UART3_BAUDRATE	115200

// uncomment define to enable RX, comment to disable RX
#ifdef UART1_BAUDRATE
#define UART1_RX_BUFSIZE	10
#endif
#ifdef UART2_BAUDRATE
#define UART2_RX_BUFSIZE	10
#endif
#ifdef UART3_BAUDRATE
#define UART3_RX_BUFSIZE	10
#endif

// uncomment define to enable TX, comment to disable TX
#ifdef UART1_BAUDRATE
#define	UART1_TX_DMA_CHANNEL DMA1_Channel4
#endif
#ifdef UART2_BAUDRATE
#define	UART2_TX_DMA_CHANNEL DMA1_Channel7
#endif
#ifdef UART3_BAUDRATE
#define	UART3_TX_DMA_CHANNEL DMA1_Channel2
#endif

//BRR=(APB1+baudrate/2)/baudrate;
#define UART1_BRR_VALUE (SystemCoreClock/ UART1_BAUDRATE)
#define UART2_BRR_VALUE (SystemCoreClock/(UART2_BAUDRATE*2))
#define UART3_BRR_VALUE (SystemCoreClock/(UART3_BAUDRATE*2))
#define	UART1_RX_DMA_CHANNEL	DMA1_Channel5
#define	UART2_RX_DMA_CHANNEL	DMA1_Channel6 
#define	UART3_RX_DMA_CHANNEL	DMA1_Channel3 
#define	UART1_TX_DMA_IFCR_CTCIF DMA_IFCR_CTCIF4
#define	UART2_TX_DMA_IFCR_CTCIF DMA_IFCR_CTCIF7
#define	UART3_TX_DMA_IFCR_CTCIF DMA_IFCR_CTCIF2

/* Types ------------------------------------- */
typedef enum {
#ifdef UART1_BAUDRATE
		DT_UART1 = 0,
	#ifdef UART2_BAUDRATE
    DT_UART2,
	#endif
	#ifdef UART3_BAUDRATE
    DT_UART3,
	#endif
#else 
	#ifdef UART2_BAUDRATE
		DT_UART2 = 0,
		#ifdef UART3_BAUDRATE
    DT_UART3,
		#endif
	#else
		DT_UART3 = 0,
	#endif
#endif
    DT_UART_COUNT
} dt_uart_t;

/* Public functions -------------------------- */

/* Public variables -------------------------- */

/* Functions --------------------------------- */
dt_status_t dt_uart_init(dt_uart_t uart);
uint16_t    dt_uart_get_rx_count(dt_uart_t uart);	// return 0 if no data, or not valid UART
uint16_t		dt_uart_read(dt_uart_t uart, uint8_t* buf, uint16_t size);
bool				dt_uart_busy(dt_uart_t uart);					// return: false - free, true - busy
dt_status_t dt_uart_write(dt_uart_t uart, const uint8_t* buf, uint16_t size);

#ifdef __cplusplus
}
#endif
#endif /* DT_UART_ */
