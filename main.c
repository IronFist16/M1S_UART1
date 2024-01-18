#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>

/* bl808 c906 std driver */
#include <bl808_glb.h>
#include <bl808_uart.h>

#define PIN_TX GLB_GPIO_PIN_6
#define PIN_RX GLB_GPIO_PIN_7
#define PIN_RTS GLB_GPIO_PIN_24
#define PIN_CTS GLB_GPIO_PIN_25
#define UART_BAUD (9600)
#define PIN_DEBUG GLB_GPIO_PIN_30
#define TX_BUF_LEN 16

#define FIFO_OF_PULSE_DURATION 25
#define FIFO_UF_PULSE_DURATION 50
#define TX_BUSY_PULSE_DURATION 75
#define TX_SENDING_PULSE_DURATION 100

//#define UART1_BASE_TRUE                  ((uint32_t)0x2000a100)
#define UART1_BASE_TRUE  UART1_BASE

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
    GLB_GPIO_Init(&cfg);

}

static void gpio_output_debug(GLB_GPIO_Type pin, uint16_t pulseDuration)
{
    GLB_GPIO_Write(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(250));
    GLB_GPIO_Write(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(pulseDuration));
    GLB_GPIO_Write(pin, 0);
}


static void uart_gpio_init_v1(uint8_t id, uint8_t tx_pin, uint8_t rx_pin, uint8_t cts_pin, uint8_t rts_pin)
{
    GLB_GPIO_Cfg_Type cfg;
    //uint8_t uart_func;

    uint32_t tmpVal;   

    vTaskDelay(500);
    GLB_UART_Sig_Swap_Set(GLB_UART_SIG_SWAP_GRP_GPIO0_GPIO11, 1);

    cfg.pullType = GPIO_PULL_UP;
    cfg.drive = 0;
    cfg.smtCtrl = 1;

    cfg.gpioMode = GPIO_MODE_AF;
    cfg.gpioFun = GPIO_FUN_UART;
    cfg.gpioPin = tx_pin;
    //uart_func = uart_func_get(id, GLB_UART_SIG_FUN_UART1_TXD);
    GLB_UART_Fun_Sel(GLB_UART_SIG_6, GLB_UART_SIG_FUN_UART1_TXD);
    //GLB_UART_Fun_Sel((GLB_UART_SIG_Type)GLB_UART_SIG_FUN_UART1_TXD, (GLB_UART_SIG_FUN_Type)GLB_UART_SIG_6);
    GLB_GPIO_Init(&cfg);
    //uart_func = uart_func_get(id, GLB_UART_SIG_FUN_UART1_RXD);
    cfg.gpioPin = rx_pin;
    GLB_UART_Fun_Sel(GLB_UART_SIG_7, GLB_UART_SIG_FUN_UART1_RXD);   
    //GLB_UART_Fun_Sel((GLB_UART_SIG_Type)GLB_UART_SIG_FUN_UART1_RXD, (GLB_UART_SIG_FUN_Type)GLB_UART_SIG_7); 
    GLB_GPIO_Init(&cfg);
    tmpVal = BL_RD_REG(GLB_BASE, GLB_UART_CFG1);
    printf("Value of UART1_CFG = 0x%08X\r\n", tmpVal);
    printf("INIT GPIO UART PINS TX & RX...[OK]\r\n");

}

void debug_uart_cr()
{
    printf("-------------------------UART CFG--------------\r\n");
    printf("GLB_BASE = 0x%08X\r\n", GLB_BASE);
    printf("Value of UART_CFG0 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_UART_CFG0));
    printf("Value of UART_CFG1 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_UART_CFG1));
    printf("Value of UART_CFG2 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_UART_CFG2));
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
    printf("GLB_GPIO_CFG0 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_UART_CFG0));
    printf("GLB_GPIO_CFG1 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_UART_CFG1));
    printf("GLB_GPIO_CFG2 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_UART_CFG2));

    printf("-------------------------GPIO CFG--------------\r\n");
    printf("GLB_GPIO_CFG0 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG0));
    printf("GLB_GPIO_CFG1 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG1));
    printf("GLB_GPIO_CFG2 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG2));
    printf("GLB_GPIO_CFG3 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG3));
    printf("GLB_GPIO_CFG4 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG4));
    printf("GLB_GPIO_CFG5 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG5));
    printf("GLB_GPIO_CFG6 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG6));
    printf("GLB_GPIO_CFG7 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG7));
    printf("GLB_GPIO_CFG8 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG8));
    printf("GLB_GPIO_CFG9 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG9));
    printf("GLB_GPIO_CFG10 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG10));
    printf("GLB_GPIO_CFG11 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG11));
    printf("GLB_GPIO_CFG12 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG12));
    printf("GLB_GPIO_CFG13 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG13));
    printf("GLB_GPIO_CFG14 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG14));
    printf("GLB_GPIO_CFG15 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG15));
    printf("GLB_GPIO_CFG16 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG16));
    printf("GLB_GPIO_CFG17 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG17));
    printf("GLB_GPIO_CFG18 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG18));
    printf("GLB_GPIO_CFG19 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG19));
    printf("GLB_GPIO_CFG20 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG20));
    printf("GLB_GPIO_CFG21 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG21));
    printf("GLB_GPIO_CFG22 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG22));
    printf("GLB_GPIO_CFG23 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG23));
    printf("GLB_GPIO_CFG24 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG24));
    printf("GLB_GPIO_CFG25 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG25));
    printf("GLB_GPIO_CFG26 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG26));
    printf("GLB_GPIO_CFG27 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG27));
    printf("GLB_GPIO_CFG28 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG28));
    printf("GLB_GPIO_CFG29 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG29));
    printf("GLB_GPIO_CFG30 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG30));
    printf("GLB_GPIO_CFG31 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG31));
    printf("GLB_GPIO_CFG32 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG32));
    printf("GLB_GPIO_CFG33 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG33));
    printf("GLB_GPIO_CFG34 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG34));
    printf("GLB_GPIO_CFG35 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG35));
    printf("GLB_GPIO_CFG36 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG36));
    printf("GLB_GPIO_CFG37 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG37));
    printf("GLB_GPIO_CFG38 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG38));
    printf("GLB_GPIO_CFG39 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG39));
    printf("GLB_GPIO_CFG40 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG40));
    printf("GLB_GPIO_CFG41 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG41));
    printf("GLB_GPIO_CFG42 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG42));
    printf("GLB_GPIO_CFG43 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG43));
    printf("GLB_GPIO_CFG44 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG44));
    printf("GLB_GPIO_CFG45 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG45));
    printf("GLB_GPIO_CFG46 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG46));
    printf("GLB_GPIO_CFG47 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG47));
    printf("GLB_GPIO_CFG48 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG48));
    printf("GLB_GPIO_CFG49 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG49));
    printf("GLB_GPIO_CFG50 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG50));
    printf("GLB_GPIO_CFG51 = 0x%08X\r\n", BL_RD_REG(GLB_BASE, GLB_GPIO_CFG51));
}


static void uart0_init(UART_ID_Type uart_id)
{
    UART_CFG_Type cfg = {
        .uartClk = (160*1000*1000),
        .baudRate = 9600,
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
    UART_IntMask(uart_id, UART_INT_ALL, MASK);
    /* disable uart before config */    
    UART_Disable(uart_id, UART_TXRX);

    uart_gpio_init_v1(uart_id, PIN_TX, PIN_RX, PIN_CTS, PIN_RTS);
    UART_Init(uart_id, &cfg);
    UART_TxFreeRun(uart_id, ENABLE);
    UART_FifoConfig(uart_id, &fifo_cfg);
    UART_Enable(uart_id, UART_TXRX);
    GPIO_init_test(PIN_DEBUG);
    gpio_test_short(PIN_DEBUG);
    printf("INIT UART1... [OK]\r\n");



}

static void uart0_test_tx(UART_ID_Type uart_id)
{

    uint8_t *buf = calloc(TX_BUF_LEN, sizeof(uint8_t));
    uint8_t *buf_rx = calloc(TX_BUF_LEN, sizeof(uint8_t));
    uint8_t send_result = SUCCESS;

    for (uint8_t i=0; i < TX_BUF_LEN; i++)
        buf[i] = 0xAB;

    //buf = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

    while(1){
        if(UART_GetOverflowStatus(uart_id, UART_TX_OVERFLOW) == SET)
        {
            printf("FIFO Overflow...\r\n");
            gpio_output_debug(PIN_DEBUG, FIFO_OF_PULSE_DURATION);
        }
        else if (UART_GetOverflowStatus(uart_id, UART_TX_UNDERFLOW) == SET)
        {
            printf("FIFO Underflow...\r\n");
            gpio_output_debug(PIN_DEBUG, FIFO_UF_PULSE_DURATION);
        }
        else if (UART_GetTxBusBusyStatus(uart_id) == SET)
        {
            printf("TxBus is Busy...\r\n");
            gpio_output_debug(PIN_DEBUG, TX_BUSY_PULSE_DURATION);
        }
        else
        {
            gpio_output_debug(PIN_DEBUG, TX_SENDING_PULSE_DURATION);
            printf("UART TX FIFO COUNT = %d \r\n", UART_GetTxFifoCount(uart_id));
            printf("FIFO_WDATA CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_FIFO_WDATA));
            printf("UTX CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_UTX_CONFIG));
            printf("Sending...");
            send_result = UART_SendData(uart_id, buf, TX_BUF_LEN);

            if (send_result == SUCCESS)
            {
                printf("[SUCCESS]\r\n");
                printf("UTX CR = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_UTX_CONFIG));
                printf("Reading Rxed Data...");
                if (UART_ReceiveData(uart_id, buf_rx, TX_BUF_LEN) == TX_BUF_LEN)
                {
                    printf("[SUCESS]\r\n");
                    printf("Data Received = 0x%08X\r\n",  *buf_rx);
                }
                else
                {
                    printf("Data not fully Received = 0x%08X\r\n",  *buf_rx);
                }
            }
            else if (send_result == TIMEOUT)    
            {
                printf("[TIMEOUT]\r\n");
                printf("UART TX FIFO COUNT = %d \r\n", UART_GetTxFifoCount(uart_id));
                printf("FIFO_WDATA CR Config = 0x%08X\r\n", BL_RD_REG(UART1_BASE, UART_FIFO_WDATA));
            }
            else 
            {
                printf("Other issue\r\n");
            }

            UART_TxFifoClear(uart_id);
        }
        vTaskDelay(1000);
    }

}


void main()
{
    uint8_t list_pins[] = {6, 7, 8, 33, 14, 15, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23,
                        41, 40, 11, 12 ,13, 18, 19, 20, 21, 22};
    uint8_t current_pin;
    bl_uart_gpio_init(UART1_ID, PIN_TX, PIN_RX, 0, 0, UART_BAUD);
    uart0_init(UART1_ID);
    debug_uart_cr();
    uart0_test_tx(UART1_ID);

    // for (uint8_t i = 0; i < sizeof(list_pins)/sizeof(list_pins[0]); i++)
    // {
    //     current_pin = list_pins[i];
    //     GPIO_init_test((GLB_GPIO_Type)current_pin);
    //     printf("Testing Pin %d...", current_pin);
    //     gpio_test_short((GLB_GPIO_Type)current_pin);
    //     vTaskDelay(500);
    //     printf("[OK]\r\n");
    //     vTaskDelay(2000);
    // }

}

