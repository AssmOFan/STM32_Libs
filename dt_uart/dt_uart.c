#include <stddef.h>
#include "dt_uart.h"
#include "dt_dma.h"

/* Variables ----------------------------------- */
#ifdef UART1_RX_BUFSIZE
static		uint8_t		uart1_rx_buff[UART1_RX_BUFSIZE] = {0};
static		uint16_t	uart1_rx_buff_tail = 0;
volatile	uint16_t	uart1_rx_buff_head = 0;
volatile	bool			uart1_rx_comp = false;
#endif
#ifdef UART2_RX_BUFSIZE
static		uint8_t		uart2_rx_buff[UART2_RX_BUFSIZE] = {0};
static		uint16_t	uart2_rx_buff_tail = 0;
volatile	uint16_t	uart2_rx_buff_head = 0;
volatile	bool			uart2_rx_comp = false;
#endif
#ifdef UART3_RX_BUFSIZE
static		uint8_t		uart3_rx_buff[UART3_RX_BUFSIZE] = {0};
static		uint16_t	uart3_rx_buff_tail = 0;
volatile	uint16_t	uart3_rx_buff_head = 0;
volatile	bool			uart3_rx_comp = false;
#endif
#ifdef UART1_TX_DMA_CHANNEL
volatile	bool			uart1_tx_busy = false;
#endif
#ifdef UART2_TX_DMA_CHANNEL
volatile	bool			uart2_tx_busy = false;
#endif
#ifdef UART3_TX_DMA_CHANNEL
volatile	bool			uart3_tx_busy = false;
#endif

/* Functions ----------------------------------- */
void config_UART_RX_DMA(USART_TypeDef* uart, DMA_Channel_TypeDef* channel, uint8_t* ptr, uint16_t size);
void config_UART_TX_DMA(USART_TypeDef* uart, DMA_Channel_TypeDef* channel, uint8_t* ptr, uint16_t size);

/* IRQHandlers --------------------------------- */
#ifdef UART1_RX_BUFSIZE
void USART1_IRQHandler(void)
{	
	/* UART RX IDLE interrupt */
  if (USART1->SR & USART_SR_IDLE){
		// update head position
		uart1_rx_buff_head = sizeof(uart1_rx_buff) - DMA_GetCurrentDataCounter(UART1_RX_DMA_CHANNEL);	
		uart1_rx_comp = true;
		// clear IDLE flag by reading the data register
		uint8_t temp = USART1->DR;
  }
}
#endif
#ifdef UART2_RX_BUFSIZE
void USART2_IRQHandler(void)
{
	/* UART RX IDLE interrupt */
  if (USART2->SR & USART_SR_IDLE){
		// update head position
		uart2_rx_buff_head = sizeof(uart2_rx_buff) - DMA_GetCurrentDataCounter(UART2_RX_DMA_CHANNEL);
		uart2_rx_comp = true;
		// clear IDLE flag by reading the data register
		uint8_t temp = USART2->DR;
		return;
  }
}
#endif
#ifdef UART3_RX_BUFSIZE
void USART3_IRQHandler(void)
{
	/* UART RX IDLE interrupt */
  if (USART3->SR & USART_SR_IDLE){
		// update head position
		uart3_rx_buff_head = sizeof(uart3_rx_buff) - DMA_GetCurrentDataCounter(UART3_RX_DMA_CHANNEL);
		uart3_rx_comp = true;
		// clear IDLE flag by reading the data register
		uint8_t temp = USART3->DR;
		return;
  }
}
#endif
#ifdef UART1_TX_DMA_CHANNEL
void DMA1_Channel4_IRQHandler(void)
{
	/* UART DMA TC interrupt */
	if (DMA1->ISR & DMA_ISR_TCIF4){
		uart1_tx_busy = false;
		DMA1->IFCR = UART1_TX_DMA_IFCR_CTCIF;
	}		
}
#endif
#ifdef UART2_TX_DMA_CHANNEL
void DMA1_Channel7_IRQHandler(void)
{
	/* UART DMA TC interrupt */
	if (DMA1->ISR & DMA_ISR_TCIF7){
		uart2_tx_busy = false;
		DMA1->IFCR = UART2_TX_DMA_IFCR_CTCIF;
	}
}
#endif
#ifdef UART3_TX_DMA_CHANNEL
void DMA1_Channel2_IRQHandler(void)
{
	/* UART DMA TC interrupt */
	if (DMA1->ISR & DMA_ISR_TCIF2){
		uart3_tx_busy = false;
		DMA1->IFCR = UART3_TX_DMA_IFCR_CTCIF;
	}
}
#endif

/* Library code ----------------------------------- */
dt_status_t dt_uart_init(dt_uart_t uart)
{
	if (uart < DT_UART_COUNT){	
		#ifdef UART1_BAUDRATE
		if (uart == DT_UART1){		
			RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;													// USART1 Clock ON
			USART1->BRR 	=  UART1_BRR_VALUE;
			USART1->CR1 	|= USART_CR1_UE;																	// USART1 ON	
			#ifdef UART1_TX_DMA_CHANNEL
			GPIOA->CRH		&= ~GPIO_CRH_CNF9;																// Clear CNF bit 9
			GPIOA->CRH		|= GPIO_CRH_CNF9_1;																// Set CNF bit 9 to 10 - AFIO Push-Pull
			GPIOA->CRH		|= GPIO_CRH_MODE9_0;															// Set MODE bit 9 to Mode 01 = 10MHz 
			USART1->CR1 	|= USART_CR1_TE;																	// TX1 ON
			NVIC_EnableIRQ(DMA1_Channel4_IRQn);
			#endif		
			#ifdef UART1_RX_BUFSIZE
			GPIOA->CRH		&= ~GPIO_CRH_CNF10;																// Clear CNF bit 10
			GPIOA->CRH		|= GPIO_CRH_CNF10_0;															// Set CNF bit 10 to 01 HiZ
			GPIOA->CRH		&= ~GPIO_CRH_MODE10;															// Set MODE bit 10 to Mode 01 = 10MHz 
			USART1->CR1 	|= USART_CR1_RE |	USART_CR1_IDLEIE;								// RX1 ON, IDLE1 Int ON									 
			config_UART_RX_DMA(USART1, UART1_RX_DMA_CHANNEL, uart1_rx_buff, sizeof(uart1_rx_buff));
			NVIC_EnableIRQ(USART1_IRQn);
			#endif			
		}
		#endif	
		#ifdef UART2_BAUDRATE
		if (uart == DT_UART2){
			RCC->APB1ENR	|= RCC_APB1ENR_USART2EN;													// USART2 Clock ON
			USART2->BRR 	=	 UART2_BRR_VALUE;
			USART2->CR1 	|= USART_CR1_UE;																	// USART2 ON
			#ifdef UART2_TX_DMA_CHANNEL
			GPIOA->CRL		&= ~GPIO_CRL_CNF2;																// Clear CNF bit 2
			GPIOA->CRL		|= GPIO_CRL_CNF2_1;																// Set CNF bit 2 to 10 - AFIO Push-Pull
			GPIOA->CRL		|= GPIO_CRL_MODE2_0;															// Set MODE bit 2 to Mode 01 = 10MHz 
			USART2->CR1 	|= USART_CR1_TE;																	// TX2 ON
			NVIC_EnableIRQ(DMA1_Channel7_IRQn);
			#endif		
			#ifdef UART2_RX_BUFSIZE
			GPIOA->CRL		&= ~GPIO_CRL_CNF3;																// Clear CNF bit 3
			GPIOA->CRL		|= GPIO_CRL_CNF3_0;																// Set CNF bit 3 to 01 HiZ
			GPIOA->CRL		&= ~GPIO_CRL_MODE3;															  // Set MODE bit 3 to Mode 01 = 10MHz 
			USART2->CR1 	|= USART_CR1_RE |	USART_CR1_IDLEIE;								// RX2 ON, IDLE2 Int ON	
			config_UART_RX_DMA(USART2, UART2_RX_DMA_CHANNEL, uart2_rx_buff, sizeof(uart2_rx_buff));
			NVIC_EnableIRQ(USART2_IRQn);
			#endif				
		}
		#endif
		#ifdef UART3_BAUDRATE
		if (uart == DT_UART3){
			RCC->APB1ENR	|= RCC_APB1ENR_USART3EN;													// USART3 Clock ON
			USART3->BRR 	=  UART3_BRR_VALUE;
			USART3->CR1 	|= USART_CR1_UE;																	// USART3 ON	
			#ifdef UART3_TX_DMA_CHANNEL
			GPIOB->CRH		&= ~GPIO_CRH_CNF10;																// Clear CNF bit 10
			GPIOB->CRH		|= GPIO_CRH_CNF10_1;															// Set CNF bit 10 to 10 - AFIO Push-Pull
			GPIOB->CRH		|= GPIO_CRH_MODE10_0;															// Set MODE bit 10 to Mode 01 = 10MHz 
			USART3->CR1 	|= USART_CR1_TE;																	// TX3 ON
			NVIC_EnableIRQ(DMA1_Channel2_IRQn);
			#endif
			#ifdef UART3_RX_BUFSIZE
			GPIOB->CRH		&= ~GPIO_CRH_CNF11;																// Clear CNF bit 11
			GPIOB->CRH		|= GPIO_CRH_CNF11_0;															// Set CNF bit 11 to 01 HiZ
			GPIOB->CRH		&= ~GPIO_CRH_MODE11;															// Set MODE bit 11 to Mode 01 = 10MHz 
			USART3->CR1 	|= USART_CR1_RE |	USART_CR1_IDLEIE;								// RX3 ON, IDLE3 Int ON
			config_UART_RX_DMA(USART3, UART3_RX_DMA_CHANNEL, uart3_rx_buff, sizeof(uart3_rx_buff));
			NVIC_EnableIRQ(USART3_IRQn);
			#endif	
		}
		#endif
		__enable_irq();
		return DT_STATUS_OK;
	}
	else {
		return DT_STATUS_INVALID_PARAMETERS;
	}
}

bool dt_uart_busy(dt_uart_t uart)
{
	if (uart < DT_UART_COUNT){
		#ifdef UART1_TX_DMA_CHANNEL
		if (uart == DT_UART1){
			return uart1_tx_busy;
		}
		#endif
		#ifdef UART2_TX_DMA_CHANNEL
		if (uart == DT_UART2){
			return uart2_tx_busy;
		}
		#endif
		#ifdef UART3_TX_DMA_CHANNEL
		if (uart == DT_UART3){
			return uart3_tx_busy;
		}
		#endif
	}
	return true;
}

dt_status_t dt_uart_write(dt_uart_t uart, const uint8_t* buf, uint16_t size)
{
	if (uart < DT_UART_COUNT || buf != NULL || size != 0){	
		#ifdef UART1_TX_DMA_CHANNEL
		if (uart == DT_UART1 && !uart1_tx_busy){
			uart1_tx_busy = true;
			config_UART_TX_DMA(USART1, UART1_TX_DMA_CHANNEL, (uint8_t*) buf, size);
			DMA1->IFCR = UART1_TX_DMA_IFCR_CTCIF;
			return DT_STATUS_OK;
		}
		#endif
		#ifdef UART2_TX_DMA_CHANNEL
		if (uart == DT_UART2 && !uart2_tx_busy){
			uart2_tx_busy = true;
			config_UART_TX_DMA(USART2, UART2_TX_DMA_CHANNEL, (uint8_t*) buf, size);
			DMA1->IFCR = UART2_TX_DMA_IFCR_CTCIF;
			return DT_STATUS_OK;
		}
		#endif
		#ifdef UART3_TX_DMA_CHANNEL
		if (uart == DT_UART3 && !uart3_tx_busy){
			uart3_tx_busy = true;
			config_UART_TX_DMA(USART3, UART3_TX_DMA_CHANNEL, (uint8_t*) buf, size);
			DMA1->IFCR = UART3_TX_DMA_IFCR_CTCIF;
			return DT_STATUS_OK;
		}
		#endif
		return DT_STATUS_BUSY;
	}
	else {
		return DT_STATUS_INVALID_PARAMETERS;
	}
}

uint16_t dt_uart_get_rx_count(dt_uart_t uart)
{
	if (uart < DT_UART_COUNT){
		#ifdef UART1_RX_BUFSIZE
		if (uart == DT_UART1 && uart1_rx_comp){
			if (uart1_rx_buff_head > uart1_rx_buff_tail){
				return uart1_rx_buff_head - uart1_rx_buff_tail;
			}
			else if (uart1_rx_buff_head < uart1_rx_buff_tail){
				return (sizeof(uart1_rx_buff) - uart1_rx_buff_tail + uart1_rx_buff_head);
			}
			else{
				return sizeof(uart1_rx_buff);
			}
		}
		#endif
		#ifdef UART2_RX_BUFSIZE
		if (uart == DT_UART2 && uart2_rx_comp){
			if (uart2_rx_buff_head > uart2_rx_buff_tail){
				return uart2_rx_buff_head - uart2_rx_buff_tail;
			}
			else if (uart2_rx_buff_head < uart2_rx_buff_tail){
				return (sizeof(uart2_rx_buff) - uart2_rx_buff_tail + uart2_rx_buff_head);
			}
			else{
				return sizeof(uart2_rx_buff);
			}
		}
		#endif
		#ifdef UART3_RX_BUFSIZE
		if (uart == DT_UART3 && uart3_rx_comp){
			if (uart3_rx_buff_head > uart3_rx_buff_tail){
				return uart3_rx_buff_head - uart3_rx_buff_tail;
			}
			else if (uart3_rx_buff_head < uart3_rx_buff_tail){
				return (sizeof(uart3_rx_buff) - uart3_rx_buff_tail + uart3_rx_buff_head);
			}
			else{
				return sizeof(uart3_rx_buff);
			}
		}
		#endif
	}
	return 0;
}

uint16_t dt_uart_read(dt_uart_t uart, uint8_t* buf, uint16_t size)
{
	if (uart < DT_UART_COUNT || buf != NULL || size != 0){
		uint16_t len = 0;
		#ifdef UART1_RX_BUFSIZE
		if (uart == DT_UART1 && uart1_rx_comp){		
			for (uint16_t i = 0 ; size != 0; size--, len++){
				buf[i++] = uart1_rx_buff[uart1_rx_buff_tail++];
				if (uart1_rx_buff_tail == sizeof(uart1_rx_buff)){
					uart1_rx_buff_tail = 0;
				}
			}
			uart1_rx_comp = false;
			return len;
		}
		#endif	
		#ifdef UART2_RX_BUFSIZE
		if (uart == DT_UART2 && uart2_rx_comp){
			for (uint16_t i = 0 ; size != 0; size--, len++){
				buf[i++] = uart2_rx_buff[uart2_rx_buff_tail++];
				if (uart2_rx_buff_tail == sizeof(uart2_rx_buff)){
					uart2_rx_buff_tail = 0;
				}
			}
			uart2_rx_comp = false;
			return len;
		}
		#endif
		#ifdef UART3_RX_BUFSIZE
		if (uart == DT_UART3 && uart3_rx_comp){
			for (uint16_t i = 0 ; size != 0; size--, len++){
				buf[i++] = uart3_rx_buff[uart3_rx_buff_tail++];
				if (uart3_rx_buff_tail == sizeof(uart3_rx_buff)){
					uart3_rx_buff_tail = 0;
				}
			}
			uart3_rx_comp = false;
			return len;
		}
		#endif
	}
	return 0;
}

void config_UART_TX_DMA(USART_TypeDef* uart, DMA_Channel_TypeDef* channel, uint8_t* ptr, uint16_t size)
{
	DMA_Disable(channel);
	DMA_DeInit(channel);
	DMA_Init(channel,								// Указываем канал контроллера 1 для USARTX
		(uint32_t)&(uart->DR),				// Указываем адрес регистра данных USARTX
		(uint32_t)ptr,								// Указываем адрес буфера для передачи. В него предварительно записаны данные
		size,													// Указываем размер буфера
		TransCompl_Int_Enable				|	// Прерывание по окончанию передачи включено
		HalfCompl_Int_Disable       |	// Прерывание по передаче половины посылки выключено
		TransError_Int_Disable      |	// Прерывание по ошибке выключено
		ReadMemory                  |	// Читаем из памяти (а пишем в периферию)
		CircularMode_Disable        |	// Циклический режим выключен
		PeripheralInc_Disable       |	// Адрес периферии не меняем
		MemoryInc_Enable            |	// А вот адрес памяти наоборот увеличиваем
		PDataSize_B                 |	// Данные размером в байт в периферии
		MDataSize_B                 |	// Данные размером в байт в памяти
		DMA_Priority_Low            |	// Низкий приоритет
		M2M_Disable                 );// Не из памяти в память
	uart->CR3 |= USART_CR3_DMAT;		// Подключаем канал DMA к UART, установив бит отвечающий за пинание UART-ом канала DMA
	DMA_Enable(channel);
}

void config_UART_RX_DMA(USART_TypeDef* uart, DMA_Channel_TypeDef* channel, uint8_t* ptr, uint16_t size)
{	
	// Enable DMA clock
	RCC->AHBENR	|= RCC_AHBENR_DMA1EN;
	DMA_Disable(channel);
	DMA_DeInit(channel);
	uart->CR3 |= USART_CR3_DMAR;		// Включили сигнал от приема на DMA
	DMA_Init(channel,								// UARTX RX закреплен на DMA1_ChannelY
		(uint32_t)&(uart->DR),				// Адрес откуда брать - адрес регистра DR в USARTX
		(uint32_t)ptr,								// Адрес куда класть результат
		size,													// Сколько класть
		TransCompl_Int_Disable      |	// Прерывание по окончанию выключено
		HalfCompl_Int_Disable       |	// Прерывание по половине выключено
		TransError_Int_Disable      |	// Прерывание по ошибке выключено
		ReadPerif                   |	// Читаем из периферии
		CircularMode_Enable         |	// Цикличный режим включен
		PeripheralInc_Disable       |	// Адрес периферии не увеличиваем
		MemoryInc_Enable            |	// А вот адрес примного буфера увеличиваем, перебирая байт за байтом его
		PDataSize_B                 |	// Размер данных из периферии - 1 байт
		MDataSize_B                 |	// Размер данных в памяти - 1 байт
		DMA_Priority_Low            |	// Низкий приоритет
		M2M_Disable                 );// Режим копирования память-память выключен
	DMA_Enable(channel);
}
