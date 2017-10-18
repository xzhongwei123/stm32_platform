/*
* scanner drivers
* com  uart2
* Author: xzw
* Date: 2017-10-16
**/
#include "stm32f10x_lib.h"
#include "gpio.h"
#include "gen_os.h"
#include "scanner.h"

#define  SCAN_COM    2
#if SCAN_COM == 2
#define SCAN_UART               USART2
#define SCAN_UART_PORT          GPIOA
#define SCAN_UART_RCC_GPX       RCC_APB2Periph_GPIOA
#define SCAN_UART_RCC           RCC_APB1Periph_USART2
#define SCAN_UART_RX_PIN        GPIO_Pin_3
#define SCAN_UART_TX_PIN        GPIO_Pin_2
#define SCAN_UART_IRQCHANNEL    USART2_IRQChannel
#define scanUart_isr            USART2_IRQHandler
#endif

#define  TEST_COM    1
#if TEST_COM == 1
#define testCOM                USART1
#define TEST_COM_PORT          GPIOA
#define TEST_COM_RCC_GPX       RCC_APB2Periph_GPIOA
#define TEST_COM_RCC           RCC_APB2Periph_USART1
#define TEST_COM_RX_PIN        GPIO_Pin_10
#define TEST_COM_TX_PIN        GPIO_Pin_9
#define TEST_COM_IRQCHANNEL    USART1_IRQChannel
#define testCom_isr            USART1_IRQHandler
#endif

unsigned char RxBuf[32];
void gpioConfig(void)
{
    gpio_init(SCAN_UART_PORT,
              SCAN_UART_TX_PIN, GPIO_Speed_50MHz, GPIO_Mode_AF_PP, SCAN_UART_RCC_GPX);
    gpio_init(SCAN_UART_PORT,
              SCAN_UART_RX_PIN, GPIO_Speed_50MHz, GPIO_Mode_IPU, 0);
}

void uartInit(unsigned BaudRate)
{
    USART_InitTypeDef  USART_InitStructure;
    NVIC_InitTypeDef     NVIC_InitStructure;
 #if COM != 1
    RCC_APB1PeriphClockCmd(SCAN_UART_RCC, ENABLE);
 #else
    RCC_APB2PeriphClockCmd(SCAN_UART_RCC, ENABLE);
 #endif
    USART_InitStructure.USART_BaudRate = BaudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Configure the STD_UART_NR */
    USART_Init(SCAN_UART, &USART_InitStructure);

    /* Enable UART interrupt IRQ channel */
    NVIC_InitStructure.NVIC_IRQChannel = SCAN_UART_IRQCHANNEL;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable recv irq*/
    USART_ITConfig(SCAN_UART, USART_IT_RXNE, ENABLE);

    /* Enable the STD_UART_NR */
    USART_Cmd(SCAN_UART, ENABLE);
    
}

static GenQueId qid;

int scanUart_Init (int BaudRate)
{

    gpioConfig();
    uartInit(BaudRate);
    if (GenQueCreate(NULL, 192, &qid) < 0)
    {
        qid = 0;
        return -1;
    }
    return 0;
}
void scanUart_sendData(int ch)
{
    USART_SendData(SCAN_UART, (char) ch);
    /* Loop until the end of transmission */
    while(USART_GetFlagStatus(SCAN_UART, USART_FLAG_TXE) == RESET)
    {
        ;
    }
}

void scanUart_isr(void)
{
    unsigned int key;
    GenOSEnterISR();
    if (USART_GetFlagStatus(SCAN_UART, USART_FLAG_RXNE) != RESET)
    {
        key = (USART_ReceiveData(SCAN_UART));
        GenQuePost(qid, (void *)key);
    }
    GenOSExitISR();
}

int scanUart_peekData(void)
{
    unsigned int key;
    int ret;
    ret = GenQueAccess(qid, (void **)&key);
    if (ret >= 0)
        return key;
    return -1;
}
int scanUart_getKey(int tmout)
{
    unsigned int key = -1;
    if (tmout < 0) 
    {
        return scanUart_peekData();
    }
    GenQuePend(qid, (void *)&key, tmout);
    return  (int)key;
}

int scanner_test(void)
{
    int ch, idx = 0;
    do
    {
        ch = scanUart_getKey(0);
        RxBuf[idx++] = (unsigned char) ch;
        if(ch < 0)
        return -1;
    }
    while(ch != '\n' || ch != '\r');
    
    USART_SendData(USART1, RxBuf[idx]);
    /* Loop until the end of transmission */
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
        ;
    }
}
