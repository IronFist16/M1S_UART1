#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>

/* bl808 c906 std driver */
#include <bl808_glb.h>
#include "bl808_uart.h"

#define PIN_TX GLB_GPIO_PIN_26
#define PIN_RX GLB_GPIO_PIN_27
#define PIN_DEBUG GLB_GPIO_PIN_30
#define TX_BUF_LEN 16

#define FIFO_OF_PULSE_DURATION 25
#define FIFO_UF_PULSE_DURATION 50
#define TX_BUSY_PULSE_DURATION 75
#define TX_SENDING_PULSE_DURATION 100

static void GPIO_init_test(GLB_GPIO_Type pin)
{
    GLB_GPIO_Cfg_Type cfg;

    cfg.pullType = GPIO_PULL_UP;
    cfg.drive = 0;
    cfg.smtCtrl = 1;

    cfg.outputMode = GPIO_OUTPUT_VALUE_MODE;
    cfg.gpioMode = GPIO_MODE_OUTPUT;
    cfg.gpioFun = GPIO_FUN_GPIO;
    cfg.gpioPin = pin;
    //cfg.gpioMode = GPIO_OUTPUT_VALUE_MODE;
    //cfg.gpioMode = GPIO_FUN_GPIO;
    GLB_GPIO_Init(&cfg);

}

static void gpio_test(GLB_GPIO_Type pin)
{
    printf("Hello\r\n");
    GPIO_init_test(pin);
    while(1)
    {
        GLB_GPIO_Write(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
        GLB_GPIO_Write(pin, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    };

}

static void gpio_test_short(GLB_GPIO_Type pin)
{
    //printf("Hello\r\n");
    GPIO_init_test(pin);
    GLB_GPIO_Write(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    GLB_GPIO_Write(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    GLB_GPIO_Write(pin, 0);
}

static void gpio_output_debug(GLB_GPIO_Type pin, uint16_t pulseDuration)
{
    GLB_GPIO_Write(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(250));
    GLB_GPIO_Write(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(pulseDuration));
    GLB_GPIO_Write(pin, 0);
}


static void uart_gpio_init_v1(uint8_t tx_pin, uint8_t rx_pin)
{
    GLB_GPIO_Cfg_Type cfg;

    uint32_t tmpVal;   

    vTaskDelay(500);

    cfg.pullType = GPIO_PULL_NONE;
    cfg.drive = 0;
    cfg.smtCtrl = 1;

    cfg.gpioMode = GPIO_MODE_AF;
    cfg.gpioFun = GPIO_FUN_UART;
    cfg.gpioPin = tx_pin;

    GLB_GPIO_Init(&cfg);

    cfg.gpioPin = rx_pin;
    GLB_GPIO_Init(&cfg);

    tmpVal = BL_RD_REG(GLB_BASE, GLB_UART_CFG1);
    printf("GLB_BASE = 0x%08X\r\n", GLB_BASE);
    printf("GLB_UART_CFG1_OFFSET = 0x%08X\r\n", GLB_UART_CFG1_OFFSET);
    printf("Value of UART1_CFG = 0x%08X\r\n", tmpVal);

    GLB_UART_Fun_Sel(GLB_UART_SIG_2, GLB_UART_SIG_FUN_UART1_TXD);
    tmpVal = BL_RD_REG(GLB_BASE, GLB_UART_CFG1);
    printf("Value of UART1_CFG = 0x%08X\r\n", tmpVal);
    
    GLB_UART_Fun_Sel(GLB_UART_SIG_3, GLB_UART_SIG_FUN_UART1_RXD);    
    tmpVal = BL_RD_REG(GLB_BASE, GLB_UART_CFG1);
    printf("Value of UART1_CFG = 0x%08X\r\n", tmpVal);
    
    printf("INIT GPIO UART PINS TX & RX...[OK]\r\n");

}

static void uart0_init()
{
    UART_CFG_Type cfg = {
        .uartClk = (160*1000*1000) / 4,
        .baudRate = 115200,
        .dataBits = UART_DATABITS_8,
        .stopBits = UART_STOPBITS_1,
        .parity = UART_PARITY_NONE,
        .ctsFlowControl = DISABLE,
        .rxDeglitch = DISABLE, 
        .rtsSoftwareControl = DISABLE, 
        .txSoftwareControl = DISABLE, 
        .txLinMode = DISABLE,
        .rxLinMode = DISABLE,
        .txBreakBitCnt = 0,
        .byteBitInverse = UART_LSB_FIRST
    };

    UART_FifoCfg_Type fifo_cfg = {
        .txFifoDmaThreshold = 16,
        .rxFifoDmaThreshold = 16,
        .txFifoDmaEnable = DISABLE,
        .rxFifoDmaEnable = DISABLE
    };


    /* disable all interrupt */
    UART_IntMask(UART1_ID, UART_INT_ALL, MASK);
    /* disable uart before config */    
    UART_Disable(UART1_ID, UART_TXRX);

    uart_gpio_init_v1(PIN_TX, PIN_RX);
    UART_Init(UART1_ID, &cfg);
    UART_TxFreeRun(UART1_ID, ENABLE);
    UART_FifoConfig(UART1_ID, &fifo_cfg);
    UART_Enable(UART1_ID, UART_TXRX);
    UART_TxFifoClear(UART1_ID);
    GPIO_init_test(PIN_DEBUG);
    gpio_test_short(PIN_DEBUG);


    printf("INIT UART1... [OK]\r\n");


    printf("-------------------------UART CFG--------------\r\n");
    printf("UTX CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_UTX_CONFIG));
    printf("URX CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_URX_CONFIG));
    printf("PRD CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_BIT_PRD));
    printf("DATA CONFIG CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_DATA_CONFIG));
    printf("IR POS. CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_UTX_IR_POSITION));
    printf("URX RTO Timer CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_URX_RTO_TIMER));
    printf("SW MODE CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_SW_MODE));
    printf("INT STATUS CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_INT_STS));
    printf("INT MASK CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_INT_MASK));
    printf("INT CLEAR CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_INT_CLEAR));
    printf("INT ENABLE CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_INT_EN));
    printf("UART STATUS CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_STATUS));
    printf("FIFO_0 CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_FIFO_CONFIG_0));
    printf("FIFO_1 CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_FIFO_CONFIG_1));

    printf("-------------------------GPIO CFG--------------\r\n");
    printf("GLB_GPIO_CFG4 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG4));
    printf("GLB_GPIO_CFG5 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG5));
    printf("GLB_GPIO_CFG14 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG14));
    printf("GLB_GPIO_CFG15 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG15));
    printf("GLB_GPIO_CFG16 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG16));
    printf("GLB_GPIO_CFG17 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG17));
    printf("GLB_GPIO_CFG26 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG26));
    printf("GLB_GPIO_CFG27 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG27));
    //printf("GLB_GPIO_CFG0 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG0));
    //printf("GLB_GPIO_CFG0 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG0));
}

static void uart0_test_tx()
{

    uint8_t *buf = calloc(TX_BUF_LEN, sizeof(uint8_t));
    uint8_t send_result = SUCCESS;

    for (uint8_t i=0; i < TX_BUF_LEN; i++)
        buf[i] = 0xAB;

    //buf = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

    while(1){
        if(UART_GetOverflowStatus(UART1_ID, UART_TX_OVERFLOW) == SET)
        {
            printf("FIFO Overflow...\r\n");
            gpio_output_debug(PIN_DEBUG, FIFO_OF_PULSE_DURATION);
        }
        else if (UART_GetOverflowStatus(UART1_ID, UART_TX_UNDERFLOW) == SET)
        {
            printf("FIFO Underflow...\r\n");
            gpio_output_debug(PIN_DEBUG, FIFO_UF_PULSE_DURATION);
        }
        else if (UART_GetTxBusBusyStatus(UART1_ID) == SET)
        {
            printf("TxBus is Busy...\r\n");
            gpio_output_debug(PIN_DEBUG, TX_BUSY_PULSE_DURATION);
        }
        else
        {
            printf("Sending...");
            gpio_output_debug(PIN_DEBUG, TX_SENDING_PULSE_DURATION);
            send_result = UART_SendData(UART1_ID, buf, TX_BUF_LEN);

            if (send_result == SUCCESS)
            {
                printf("[SUCCESS]\r\n");
            }
            else if (send_result == TIMEOUT)
            {
                printf("[TIMEOUT]\r\n");
                printf("UART TX FIFO COUNT = %d \r\n", UART_GetTxFifoCount(UART1_ID));
                printf("FIFO_WDATA CR Config = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_FIFO_WDATA));
            }
            else 
            {
                printf("Other issue\r\n");
            }

            UART_TxFifoClear(UART1_ID);
        }
        vTaskDelay(1000);
    }

}


void main()
{
    printf("Hello\r\n");
    vTaskDelay(1000);
    uart0_init();
    uart0_test_tx();
}

