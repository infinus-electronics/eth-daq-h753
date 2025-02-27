/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "NetworkInterface.h"

#include "UDPLoggingPrintf.h"
#include "eventLogging.h"
#include "hr_gettime.h"
#include "tcp_mem_stats.h"

#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define mainTCP_SERVER_STACK_SIZE 640
#define USE_ZERO_COPY 1
#define CONTINUOUS_PING 0
#define ADC_BUFFER_HALF_SIZE 32768
#define AUX_ADC_BUFFER_HALF_SIZE 16384
#define TC_ADC_BUFFER_HALF_SIZE 64
#define CMD_BUFFER_SIZE 128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined(__ICCARM__) /*!< IAR Compiler */
#pragma location = 0x30000000
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location = 0x30000080
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined(__CC_ARM) /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000080))) ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined(__GNUC__) /* GNU Compiler */
//
// ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
// ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

// ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c4;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim1_up;
DMA_HandleTypeDef hdma_tim3_up;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA_HandleTypeDef hdma_tim4_up;
DMA_HandleTypeDef hdma_tim5_up;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//identifier constants
const char ucFirmwareVersion[] = "v0.3.5";
const char ucHardwareVersion[] = "v2";
const uint32_t ulSysTimClock = 200000000UL;
uint32_t ulADCSR = 0;
uint32_t ulAuxADCSR = 0;
uint32_t ulTCADCSR = 0;


// FreeRTOS and Networking
const uint8_t ucIPAddress[4] = {configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3};
const uint8_t ucNetMask[4] = {configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3};
const uint8_t ucGatewayAddress[4] = {configGATEWAY_ADDR0, configGATEWAY_ADDR1, configGATEWAY_ADDR2, configGATEWAY_ADDR3};
const uint8_t ucDNSServerAddress[4] = {configDNS_SERVER_ADDR0, configDNS_SERVER_ADDR1, configDNS_SERVER_ADDR2, configDNS_SERVER_ADDR3};
uint8_t ucMACAddress[6] = {configMAC_ADDR0, configMAC_ADDR1, configMAC_ADDR2, configMAC_ADDR3, configMAC_ADDR4, configMAC_ADDR5};
const uint8_t ucServerIPAddress[4] = {192, 168, 1, 3};
const uint16_t usADCPort = 5555;
const uint16_t usAuxADCPort = 5556;
const uint16_t usTCADCPort = 5557;
const uint16_t usCommandPort = 5001;
const uint16_t usHandshakePort = 5002;

/* There is only 1 physical interface. */
NetworkInterface_t xInterfaces[1];
/* It will have several end-points. */
NetworkEndPoint_t xEndPoints[4];

static BaseType_t xTasksAlreadyCreated = pdFALSE;
static BaseType_t xDoCreateSockets;
static uint32_t ulSeed;

static TaskHandle_t xServerWorkTaskHandle = NULL;
static TaskHandle_t vNotifierTaskHandle = NULL;
TaskHandle_t vADCTCPTaskHandle = NULL;
TaskHandle_t vAuxADCTCPTaskHandle = NULL;
TaskHandle_t vTCADCTCPTaskHandle = NULL;
static TaskHandle_t vCommandServerTaskHandle = NULL;
static TaskHandle_t vHandshakeTaskHandle = NULL;

const UBaseType_t xADCNotifyIndex = 0; // needs configuring

// HS ADC
uint16_t usZero __attribute__((section(".ram2_data"))) = 0; // DMA cannot access DTCM, so declare here
uint16_t usHSADCData0[ADC_BUFFER_HALF_SIZE] __attribute__((section(".ram2_data")));
uint16_t usHSADCData1[ADC_BUFFER_HALF_SIZE] __attribute__((section(".ram2_data")));
// GADC
uint16_t usGADCData0[AUX_ADC_BUFFER_HALF_SIZE] __attribute__((section(".ram2_data")));
uint16_t usGADCData1[AUX_ADC_BUFFER_HALF_SIZE] __attribute__((section(".ram2_data")));
// TC ADC
uint16_t usTCADCConfig[2] __attribute__((section(".ram2_data")));
uint16_t usTCADCData0[TC_ADC_BUFFER_HALF_SIZE] __attribute__((section(".ram2_data")));
uint16_t usTCADCData1[TC_ADC_BUFFER_HALF_SIZE] __attribute__((section(".ram2_data")));

// 7 Segment Display
uint32_t ulSevenSegD1 __attribute__((section(".ram2_data"))) = 0xFF0000;
uint32_t ulSevenSegD2 __attribute__((section(".ram2_data"))) = 0xFF0000; // display 8.8.

uint32_t ulUID[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
// static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */
static void vHeapInit(void);
extern void vStartHighResolutionTimer(void);

extern NetworkInterface_t *pxSTM32H_FillInterfaceDescriptor(BaseType_t xEMACIndex, NetworkInterface_t *pxInterface);

static void prvServerWorkTask(void *pvParameters);
static void vNotifierTask(void *pvParameters);
static void vADCTCPTask(void *pvParameters);
static void vAuxADCTCPTask(void *pvParameters);
static void vTCADCTCPTask(void *pvParameters);
static void vCommandServerTask(void *pvParameters);
static void prvCommandHandlerTask(void *pvParameters);
static void vHandshakeTask(void *pvParameters);
void prvGenerateMACFromUID(const uint32_t *uid_96bit, uint8_t *mac_addr);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  usZero = 0;

  memset(usHSADCData0, 0x41, sizeof(usHSADCData0));
  memset(usHSADCData1, 0x42, sizeof(usHSADCData1));
  memset(usGADCData0, 0x43, sizeof(usGADCData0));
  memset(usGADCData1, 0x44, sizeof(usGADCData1));
  __DSB();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  vHeapInit();
  vStartHighResolutionTimer();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  //  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  // MX_TIM2_Init();
  MX_I2C4_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  // Read UID
  ulUID[0] = *(unsigned int *)UID_BASE;
  ulUID[1] = *(unsigned int *)(UID_BASE + 4);
  ulUID[2] = *(unsigned int *)(UID_BASE + 8);

  // RNG Setup and Seeding
  RNG->CR &= ~RNG_CR_IE;
  RNG->CR |= RNG_CR_RNGEN;
  while ((RNG->SR & RNG_SR_DRDY) == 0)
  {
  };
  while (RNG->DR == 0)
    ; // wait for valid data
  ulSeed = RNG->DR;
  srand(ulSeed);
  // 7 Segment Setup
  // TIM4 CH1
  DMA1_Stream4->M0AR = &ulSevenSegD1;
  DMA1_Stream4->PAR = &(GPIOD->BSRR); // top 16 bits are reset, bottom 16 bits are set, set has priority if both bits set
  DMA1_Stream4->NDTR = 1;
  __DSB(); // required?
  DMA1_Stream4->CR |= DMA_SxCR_EN;
  // TIM4 UP
  DMA1_Stream5->M0AR = &ulSevenSegD2;
  DMA1_Stream5->PAR = &(GPIOD->BSRR);
  DMA1_Stream5->NDTR = 1;
  __DSB(); // required?
  DMA1_Stream5->CR |= DMA_SxCR_EN;
  //??? initializing the vars in the top just doesn't work???
  ulSevenSegD2 = 0x00FF0000;
  ulSevenSegD1 = 0x00FF0000;
  // TIM4 drives multiplexing
  TIM4->CR1 |= TIM_CR1_URS;
  TIM4->CR1 &= ~TIM_CR1_UDIS;
  TIM4->CR2 &= ~TIM_CR2_CCDS;
  TIM4->DIER |= TIM_DIER_UDE | TIM_DIER_CC1DE;
  TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM4->EGR |= TIM_EGR_UG | TIM_EGR_CC1G;
  TIM4->CR1 |= TIM_CR1_CEN;

  // DAC Setup
  HAL_GPIO_WritePin(DUT_DAC_RESET_GPIO_Port, DUT_DAC_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DUT_DAC_LDAC_GPIO_Port, DUT_DAC_LDAC_Pin, GPIO_PIN_RESET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(DUT_DAC_RESET_GPIO_Port, DUT_DAC_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(50);

  I2C4->CR2 = ((0b0001100 << 1) & 0xFFFE) // 7-bit address
              | (3 << 16)                 // NBYTES = 2
              | (0 << 10)                 // Write direction (0 = write)
              | I2C_CR2_AUTOEND           // Auto generate STOP
              | I2C_CR2_START;            // Generate START
  while ((I2C4->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)) == 0)
    ;
  //      if (I2C4->ISR & I2C_ISR_NACKF) {
  //          I2C4->ICR |= I2C_ICR_NACKCF;  // Clear NACK flag
  //          return;  // Abort on failure
  //      }
  I2C4->TXDR = 0b00110001; // write 2V = 3276 to DAC A, left justified 12 bit to 16 bit
  while ((I2C4->ISR & (I2C_ISR_TXIS)) == 0)
    ;
  I2C4->TXDR = 0x66; // MSB
  while ((I2C4->ISR & (I2C_ISR_TXIS)) == 0)
    ;
  I2C4->TXDR = 0x60; // LSB
  while ((I2C4->ISR & (I2C_ISR_TXE)) == 0)
    ;
  // Check if NACK occurred
  if (I2C4->ISR & I2C_ISR_NACKF)
  {
    // Handle error (e.g., reset I2C)
    I2C4->ICR |= I2C_ICR_NACKCF; // Clear NACK flag
  }

  I2C4->CR2 = ((0b0001100 << 1) & 0xFFFE) // 7-bit address
              | (3 << 16)                 // NBYTES = 2
              | (0 << 10)                 // Write direction (0 = write)
              | I2C_CR2_AUTOEND           // Auto generate STOP
              | I2C_CR2_START;            // Generate START
  while ((I2C4->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)) == 0)
    ;
  //      if (I2C4->ISR & I2C_ISR_NACKF) {
  //          I2C4->ICR |= I2C_ICR_NACKCF;  // Clear NACK flag
  //          return;  // Abort on failure
  //      }
  I2C4->TXDR = 0b00111000; // write 0.4V = 656 to DAC B, left justified 12 bit to 16 bit
  while ((I2C4->ISR & (I2C_ISR_TXIS)) == 0)
    ;
  I2C4->TXDR = 0x29; // MSB
  while ((I2C4->ISR & (I2C_ISR_TXIS)) == 0)
    ;
  I2C4->TXDR = 0x00; // LSB
  while ((I2C4->ISR & (I2C_ISR_TXE)) == 0)
    ;
  // Check if NACK occurred
  if (I2C4->ISR & I2C_ISR_NACKF)
  {
    // Handle error (e.g., reset I2C)
    I2C4->ICR |= I2C_ICR_NACKCF; // Clear NACK flag
  }

  // SPI1 RX Stream
  DMA1_Stream0->M0AR = usHSADCData0;
  DMA1_Stream0->M1AR = usHSADCData1;
  DMA1_Stream0->PAR = &(SPI1->RXDR);
  DMA1_Stream0->CR |= DMA_DOUBLE_BUFFER_M0;
  DMA1_Stream0->NDTR = ADC_BUFFER_HALF_SIZE;
  //  DMA1_Stream0->CR |= DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
  DMA1_Stream0->CR |= DMA_SxCR_TCIE;
  __DSB(); // required?
  DMA1_Stream0->CR |= DMA_SxCR_EN;

  // SPI2 RX Stream
  DMA1_Stream1->M0AR = usGADCData0;
  DMA1_Stream1->M1AR = usGADCData1;
  DMA1_Stream1->PAR = &(SPI2->RXDR);
  DMA1_Stream1->CR |= DMA_DOUBLE_BUFFER_M0;
  DMA1_Stream1->NDTR = AUX_ADC_BUFFER_HALF_SIZE;
  //  DMA1_Stream0->CR |= DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
  DMA1_Stream1->CR |= DMA_SxCR_TCIE;
  __DSB(); // required?
  DMA1_Stream1->CR |= DMA_SxCR_EN;

  // TIM1 UP DMA
  DMA1_Stream2->M0AR = &usZero;
  DMA1_Stream2->PAR = &(SPI1->TXDR);
  DMA1_Stream2->NDTR = 1;
  __DSB(); // required?
  DMA1_Stream2->CR |= DMA_SxCR_EN;

  // TIM3 UP DMA
  DMA1_Stream3->M0AR = &usZero;
  DMA1_Stream3->PAR = &(SPI2->TXDR);
  DMA1_Stream3->NDTR = 1;
  __DSB(); // required?
  DMA1_Stream3->CR |= DMA_SxCR_EN;

  // initialize high speed ADC here
  SPI1->CR2 = 0; // reinitialize tsize
  SPI1->CR1 |= SPI_CR1_SPE;
  SPI1->CR1 |= SPI_CR1_CSTART;
  HAL_GPIO_WritePin(HS_ADC_RESET_GPIO_Port, HS_ADC_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(HS_ADC_RESET_GPIO_Port, HS_ADC_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  uint8_t spi_data[2] = {0b00011011, 0x80 + 0x05}; // low reference, low input, vcm on, refpbuf on, input buf on
  SPI1->TXDR = ((spi_data[1] << 8) | spi_data[0]);
  while ((SPI1->SR & SPI_SR_TXC) == 0)
  {
  }; // wait for enough space to become available
  spi_data[1]++;
  spi_data[0] = 0b00010000; // sync control mode
  SPI1->TXDR = ((spi_data[1] << 8) | spi_data[0]);
  while ((SPI1->SR & SPI_SR_TXC) == 0)
  {
  }; // wait for enough space to become available
  spi_data[1]++;
  spi_data[0] = 0b00001001; // sinc4 osr16
  SPI1->TXDR = ((spi_data[1] << 8) | spi_data[0]);
  while ((SPI1->SR & SPI_SR_TXC) == 0)
  {
  }; // wait for enough space to become available
  spi_data[1]++;
  spi_data[0] = 0b10000000; // external clock
  SPI1->TXDR = ((spi_data[1] << 8) | spi_data[0]);
  while ((SPI1->SR & SPI_SR_TXC) == 0)
  {
  }; // wait for enough space to become available
  spi_data[1] = 0x80 + 0x03; // start conversion
  spi_data[0] = 0b00000010;
  SPI1->TXDR = ((spi_data[1] << 8) | spi_data[0]);
  while ((SPI1->SR & SPI_SR_TXC) == 0)
  {
  }; // wait for enough space to become available
  SPI1->CR1 &= ~SPI_CR1_SPE;

  // GADC Setup
  SPI2->CR2 = 0;         // reinitialize tsize
  SPI2->CFG1 |= 0b11111; // use 32 bit mode for config
  SPI2->CR1 |= SPI_CR1_SPE;
  SPI2->CR1 |= SPI_CR1_CSTART;
  HAL_GPIO_WritePin(GADC_RESET_GPIO_Port, GADC_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GADC_RESET_GPIO_Port, GADC_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  uint16_t ucGADCSPIData[2] = {0b1101000000000100, (1 << 5) | (1 << 4)}; // disable alarms
  SPI2->TXDR = ((ucGADCSPIData[0] << 16) | ucGADCSPIData[1]);
  while ((SPI2->SR & SPI_SR_TXC) == 0)
  {
  }; // wait for enough space to become available
  ucGADCSPIData[0] = 0b1101000000010100; // 14h
  ucGADCSPIData[1] = 0b10;               // range select +-1.5x VREF
  SPI2->TXDR = ((ucGADCSPIData[0] << 16) | ucGADCSPIData[1]);
  while ((SPI2->SR & SPI_SR_TXC) == 0)
  {
  };
  SPI2->CR1 &= ~SPI_CR1_SPE;
  SPI2->CFG1 &= ~0b11111;
  SPI2->CFG1 |= 0b1111; // switch back to 16 bit transfers

  // TC ADC SETUP
  // interleave temp sensor and internal ADC
  usTCADCConfig[0] = 0b1000101110001010; // FSR 0.256mV, 128SPS, Single Conversion, Start Conversion, Default Inputs
  usTCADCConfig[1] = 0b1000101110011010; // same as above, use internal temp sensor
  // TIM5 UP
  DMA1_Stream6->M0AR = usTCADCConfig;
  DMA1_Stream6->PAR = &(SPI3->TXDR);
  DMA1_Stream6->NDTR = 2;
  __DSB(); // required?
  DMA1_Stream6->CR |= DMA_SxCR_EN;
  // SPI3 RX Stream
  DMA1_Stream7->M0AR = usTCADCData0;
  DMA1_Stream7->M1AR = usTCADCData1;
  DMA1_Stream7->PAR = &(SPI3->RXDR);
  DMA1_Stream7->CR |= DMA_DOUBLE_BUFFER_M0;
  DMA1_Stream7->NDTR = TC_ADC_BUFFER_HALF_SIZE;
  //  DMA1_Stream0->CR |= DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
  DMA1_Stream7->CR |= DMA_SxCR_TCIE;
  __DSB(); // required?
  DMA1_Stream7->CR |= DMA_SxCR_EN;
  // SPI3 Config
  SPI3->CR2 = 0; // reinitialize tsize
  SPI3->CFG1 |= SPI_CFG1_RXDMAEN;
  SPI3->CR1 |= SPI_CR1_SPE;
  SPI3->CR1 |= SPI_CR1_CSTART;

  HAL_GPIO_WritePin(DUT_GATE_SEL_GPIO_Port, DUT_GATE_SEL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DUT_HVDC_ENABLE_GPIO_Port, DUT_HVDC_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DUT_VICTRL_SEL_GPIO_Port, DUT_VICTRL_SEL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DUT_VGS_IDLE_SEL_GPIO_Port, DUT_VGS_IDLE_SEL_Pin, GPIO_PIN_RESET);

  // Enable SPI1
  //  SPI1->CR1 &=  ~SPI_CR1_SPE;
  SPI1->CR2 = 0; // reinitialize tsize
  SPI1->CFG1 |= SPI_CFG1_RXDMAEN;
  SPI1->CR1 |= SPI_CR1_SPE;
  SPI1->CR1 |= SPI_CR1_CSTART;

  // Enable SPI2
  SPI2->CR2 = 0; // reinitialize tsize
  SPI2->CFG1 |= SPI_CFG1_RXDMAEN;
  SPI2->CR1 |= SPI_CR1_SPE;
  SPI2->CR1 |= SPI_CR1_CSTART;

  // Enable TIM1 (SPI1)
  TIM1->CR1 |= TIM_CR1_URS;
  TIM1->CR1 &= ~TIM_CR1_UDIS;
  TIM1->DIER |= TIM_DMA_UPDATE;
  TIM1->EGR |= TIM_EGR_UG;
  //  TIM1->CR1 |= TIM_CR1_CEN;

  // Enable TIM3 (SPI2)
  TIM3->CR1 |= TIM_CR1_URS;
  TIM3->CR1 &= ~TIM_CR1_UDIS;
  TIM3->DIER |= TIM_DMA_UPDATE;
  TIM3->EGR |= TIM_EGR_UG;
  //  TIM3->CR1 |= TIM_CR1_CEN;

  // enable TIM5 (SPI3 TX)
  TIM5->CR1 |= TIM_CR1_URS;
  TIM5->CR1 &= ~TIM_CR1_UDIS;
  TIM5->DIER |= TIM_DMA_UPDATE;
  TIM5->EGR |= TIM_EGR_UG;
  //  TIM5->CR1 |= TIM_CR1_CEN;

  // use UID to derive MAC address
  prvGenerateMACFromUID(ulUID, ucMACAddress);
  /* Initialise the interface descriptor for WinPCap for example. */
  pxSTM32H_FillInterfaceDescriptor(0, &(xInterfaces[0]));

  FreeRTOS_FillEndPoint(&(xInterfaces[0]), &(xEndPoints[0]), ucIPAddress,
                        ucNetMask, ucGatewayAddress, ucDNSServerAddress, ucMACAddress);
#if (ipconfigUSE_DHCP != 0)
  {
    /* End-point 0 wants to use DHCPv4. */
    xEndPoints[0].bits.bWantDHCP = pdTRUE;
  }
#endif /* ( ipconfigUSE_DHCP != 0 ) */

  /* Initialise the RTOS's TCP/IP stack.  The tasks that use the network
     are created in the vApplicationIPNetworkEventHook() hook function
     below.  The hook function is called when the network connects. */
  FreeRTOS_IPInit_Multi();

  xTaskCreate(prvServerWorkTask, "SvrWork", mainTCP_SERVER_STACK_SIZE, NULL, 0, &xServerWorkTaskHandle);
  //      xTaskCreate ( vNotifierTask, "Notif", 200, NULL, 0, &vNotifierTaskHandle);
  xTaskCreate(vADCTCPTask, "HSADC_TCP", mainTCP_SERVER_STACK_SIZE, NULL, 1, &vADCTCPTaskHandle);
  xTaskCreate(vAuxADCTCPTask, "GADC_TCP", mainTCP_SERVER_STACK_SIZE, NULL, 1, &vAuxADCTCPTaskHandle);
  xTaskCreate(vTCADCTCPTask, "TCADC_TCP", mainTCP_SERVER_STACK_SIZE, NULL, 1, &vTCADCTCPTaskHandle);
  xTaskCreate(vCommandServerTask, "CommandServer", mainTCP_SERVER_STACK_SIZE, NULL, tskIDLE_PRIORITY, &vCommandServerTaskHandle);
  xTaskCreate(vHandshakeTask, "Handshake", mainTCP_SERVER_STACK_SIZE, NULL, tskIDLE_PRIORITY, &vHandshakeTaskHandle);
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3 | RCC_PERIPHCLK_SPI2 | RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 8;
  PeriphClkInitStruct.PLL2.PLL2N = 64;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
// static void MX_ETH_Init(void)
//{
//
//   /* USER CODE BEGIN ETH_Init 0 */
//////////
//  /* USER CODE END ETH_Init 0 */
//
//   static uint8_t MACAddr[6];
//
//  /* USER CODE BEGIN ETH_Init 1 */
//////////
//  /* USER CODE END ETH_Init 1 */
//  heth.Instance = ETH;
//  MACAddr[0] = 0x00;
//  MACAddr[1] = 0x80;
//  MACAddr[2] = 0xE1;
//  MACAddr[3] = 0x00;
//  MACAddr[4] = 0x00;
//  MACAddr[5] = 0x00;
//  heth.Init.MACAddr = &MACAddr[0];
//  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
//  heth.Init.TxDesc = DMATxDscrTab;
//  heth.Init.RxDesc = DMARxDscrTab;
//  heth.Init.RxBuffLen = 1524;
//
//  /* USER CODE BEGIN MACADDRESS */
//////////
//  /* USER CODE END MACADDRESS */
//
//  if (HAL_ETH_Init(&heth) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
//  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
//  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
//  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
//  /* USER CODE BEGIN ETH_Init 2 */
//////////
/* USER CODE END ETH_Init 2 */

//}

/**
 * @brief I2C4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x10C0ECFF;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */
}

/**
 * @brief RNG Initialization Function
 * @param None
 * @retval None
 */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_03CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_03CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_03CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_03CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_02CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_02CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  ulADCSR = ulSysTimClock / (htim1.Init.Prescaler + 1) / (pow(2, htim1.Init.ClockDivision)) / (htim1.Init.Period + 1);
  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  //
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  //
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  //
  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  ulAuxADCSR = ulSysTimClock / (htim3.Init.Prescaler + 1) / (pow(2, htim3.Init.ClockDivision)) / (htim3.Init.Period + 1);
  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 3124;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  ulTCADCSR = ulSysTimClock / (htim5.Init.Prescaler + 1) / (pow(2, htim5.Init.ClockDivision)) / (htim5.Init.Period + 1);
  /* USER CODE END TIM5_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DUT_HVDC_ENABLE_Pin | DUT_VGS_IDLE_SEL_Pin | DUT_VICTRL_SEL_Pin | DUT_GATE_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GADC_RESET_Pin | DUT_DAC_LDAC_Pin | DUT_DAC_RESET_Pin | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HS_ADC_START_Pin | HS_ADC_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DUT_HVDC_ENABLE_Pin DUT_VGS_IDLE_SEL_Pin DUT_VICTRL_SEL_Pin DUT_GATE_SEL_Pin */
  GPIO_InitStruct.Pin = DUT_HVDC_ENABLE_Pin | DUT_VGS_IDLE_SEL_Pin | DUT_VICTRL_SEL_Pin | DUT_GATE_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : GADC_RESET_Pin DUT_DAC_LDAC_Pin DUT_DAC_RESET_Pin PD0
                           PD1 PD2 PD3 PD4
                           PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GADC_RESET_Pin | DUT_DAC_LDAC_Pin | DUT_DAC_RESET_Pin | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GADC_RVS_Pin */
  GPIO_InitStruct.Pin = GADC_RVS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GADC_RVS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HS_ADC_START_Pin HS_ADC_RESET_Pin */
  GPIO_InitStruct.Pin = HS_ADC_START_Pin | HS_ADC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HS_ADC_DRDY_Pin */
  GPIO_InitStruct.Pin = HS_ADC_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HS_ADC_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EFUSE_FLT_Pin EFUSE_PGOOD_Pin */
  GPIO_InitStruct.Pin = EFUSE_FLT_Pin | EFUSE_PGOOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Initialize Heap for heap5

/**
 * RAM area	H747	H743	H742	Location
 * ------------------------------------------------
 * DTCM		128k	128k	128k	0x20000000
 * AXI-SRAM	511k	511k	384k	0x24000000
 *
 * SRAM1	128k	128k	32k		0x30000000
 * SRAM2	128k	128k	16k		0x30020000
 * SRAM3	32k		32k	 	-		0x30040000
 * SRAM4	64k		64k		64k		0x38000000
 * Backup   SRAM	4k		4k	4k	0x38800000
 */

static uint8_t ucRAM_dtcm[96 * 1024] __attribute__((section(".dtcm_data")));
static uint8_t ucRAM_1[256 * 1024] __attribute__((section(".ethernet_data")));
// static uint8_t ucRAM_2 [128 * 1024] __attribute__( ( section( ".ram2_data" ) ) );
static uint8_t ucRAM_3[32 * 1024] __attribute__((section(".ram3_data")));

#define mainMEM_REGION(REGION) REGION, sizeof(REGION)

static void vHeapInit()
{
  /* Note: the memories must be sorted on their physical address. */
  HeapRegion_t xHeapRegions[] = {
      {mainMEM_REGION(ucRAM_dtcm)},
      {mainMEM_REGION(ucRAM_1)},
      //		{ mainMEM_REGION( ucRAM_2 ) },
      {mainMEM_REGION(ucRAM_3)},
      {NULL, 0}};

  vPortDefineHeapRegions(xHeapRegions);
}

// helper functions

uint32_t ulGetRunTimeCounterValue()
{
  return 0U;
}

void vAssertCalled(const char *pcFile, uint32_t ulLine)
{
  volatile unsigned long ul = 0;

  (void)pcFile;
  (void)ulLine;

  taskENTER_CRITICAL();
  {
    /* Set ul to a non-zero value using the debugger to step out of this
    function. */
    while (ul == 0)
    {
      __NOP();
    }
  }
  taskEXIT_CRITICAL();
}

void vApplicationMallocFailedHook(void)
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

  /* Force an assert. */
  configASSERT((volatile void *)NULL);
}

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
  /* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static – otherwise they will be allocated on
the stack and so not exists after this function exits. */
  static StaticTask_t xIdleTaskTCB;
  static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

  /* Pass out a pointer to the StaticTask_t structure in which the Idle task’s
    state will be stored. */
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

  /* Pass out the array that will be used as the Idle task’s stack. */
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;

  /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*———————————————————–*/

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
  /* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static – otherwise they will be allocated on
the stack and so not exists after this function exits. */
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

  /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task’s state will be stored. */
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

  /* Pass out the array that will be used as the Timer task’s stack. */
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;

  /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

// Examples of the callback functions that must be provided by the application to
//
// supply the RAM used by the Idle and Timer Service tasks if configSUPPORT_STATIC_ALLOCATION
//
// is set to 1.

uint32_t ulApplicationGetNextSequenceNumber(
    uint32_t ulSourceAddress,
    uint16_t usSourcePort,
    uint32_t ulDestinationAddress,
    uint16_t usDestinationPort)
{
  uint32_t ulReturn;
  (void)ulSourceAddress;
  (void)usSourcePort;
  (void)ulDestinationAddress;
  (void)usDestinationPort;
  xApplicationGetRandomNumber(&ulReturn);

  return ulReturn;
}

void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent)
{
  /* If the network has just come up...*/
  if (eNetworkEvent == eNetworkUp)
  {
    uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
    char cBuffer[16];

    /* Create the tasks that use the IP stack if they have not already been
    created. */
    if (xTasksAlreadyCreated == pdFALSE)
    {
      xTasksAlreadyCreated = pdTRUE;
      /* Sockets, and tasks that use the TCP/IP stack can be created here. */
      xTaskNotifyGiveIndexed(vHandshakeTaskHandle, 0);
      xDoCreateSockets = pdTRUE;
    }
    /* Print out the network configuration, which may have come from a DHCP
    server. */
    FreeRTOS_GetAddressConfiguration(&ulIPAddress, &ulNetMask, &ulGatewayAddress, &ulDNSServerAddress);
    FreeRTOS_inet_ntoa(ulIPAddress, cBuffer);
    FreeRTOS_printf(("IP Address: %s\n", cBuffer));

    FreeRTOS_inet_ntoa(ulNetMask, cBuffer);
    FreeRTOS_printf(("Subnet Mask: %s\n", cBuffer));

    FreeRTOS_inet_ntoa(ulGatewayAddress, cBuffer);
    FreeRTOS_printf(("Gateway Address: %s\n", cBuffer));

    FreeRTOS_inet_ntoa(ulDNSServerAddress, cBuffer);
    FreeRTOS_printf(("DNS Server Address: %s\n", cBuffer));

    FreeRTOS_printf(("RNG Seed: %u\n", ulSeed));

    FreeRTOS_printf(("Device UID: %u-%u-%u\n", ulUID[0], ulUID[1], ulUID[2]));

    FreeRTOS_printf(("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", ucMACAddress[0], ucMACAddress[1], ucMACAddress[2], ucMACAddress[3], ucMACAddress[4], ucMACAddress[5]));
  }
}
/*-----------------------------------------------------------*/

BaseType_t xApplicationDNSQueryHook(const char *pcName)
{
  BaseType_t xReturn = pdFAIL;

  /* Determine if a name lookup is for this node.  Two names are given
  to this node: that returned by pcApplicationHostnameHook() and that set
  by mainDEVICE_NICK_NAME. */
  if (strcasecmp(pcName, pcApplicationHostnameHook()) == 0)
  {
    xReturn = pdPASS;
  }
  return xReturn;
}
/*-----------------------------------------------------------*/

const char *pcApplicationHostnameHook(void)
{
  /* Assign the name "STM32H7" to this network node.  This function will be
  called during the DHCP: the machine will be registered with an IP address
  plus this name. */
  return "STM32H7";
}
/*-----------------------------------------------------------*/

#if (ipconfigSUPPORT_OUTGOING_PINGS == 1)
void vApplicationPingReplyHook(ePingReplyStatus_t eStatus, uint16_t usIdentifier)
{
  FreeRTOS_printf(("Received ping ID %04X\n", usIdentifier));
}
#endif

/*-----------------------------------------------------------*/

BaseType_t xApplicationGetRandomNumber(uint32_t *pulNumber)
{
  *pulNumber = rand();
  return pdTRUE;
}
/*-----------------------------------------------------------*/

struct xREGISTER_STACK
{
  uint32_t spare0[8];
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;  /* Link register. */
  uint32_t pc;  /* Program counter. */
  uint32_t psr; /* Program status register. */
  uint32_t spare1[8];
};

volatile struct xREGISTER_STACK *pxRegisterStack = NULL;

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
  /* When the debuggger stops here, you can inspect the registeers of the
  application by looking at *pxRegisterStack. */
  pxRegisterStack = (volatile struct xREGISTER_STACK *)(pulFaultStackAddress - ARRAY_SIZE(pxRegisterStack->spare0));

  /* When the following line is hit, the variables contain the register values. */
  for (;;)
    ;
}

void HardFault_Handler(void)
{
  __asm volatile(
      " tst lr, #4                                                \n"
      " ite eq                                                    \n"
      " mrseq r0, msp                                             \n"
      " mrsne r0, psp                                             \n"
      " ldr r1, [r0, #24]                                         \n"
      " bl prvGetRegistersFromStack                               \n");
}

static void prvServerWorkTask(void *pvParameters)
{
#if (CONTINUOUS_PING != 0)
  /* CONTINUOUS_PING can be used while testing the network driver. */
  uint32_t ulIPAddress = FreeRTOS_inet_addr_quick(192, 168, 2, 5);
  size_t uxNumberOfBytesToSend = 16;
  TickType_t uxBlockTimeTicks = ipMS_TO_MIN_TICKS(100U);
#endif /* ( CONTINUOUS_PING != 0 ) */

  for (;;)
  {
    vTaskDelay(10U);
    if (xDoCreateSockets != pdFALSE)
    {
      xDoCreateSockets = pdFALSE;
      /* Start a new task to fetch logging lines and send them out.
      See FreeRTOSConfig.h for the configuration of UDP logging. */
      vUDPLoggingTaskCreate();
      //			FreeRTOS_printf(("Hello"));
      //			vIPerfInstall();
#if (USE_LOG_EVENT != 0)
      {
        iEventLogInit();
      }
#endif
    };

#if (CONTINUOUS_PING != 0)
    {
      if (xTasksAlreadyCreated != pdFALSE)
      {
        FreeRTOS_SendPingRequest(ulIPAddress, uxNumberOfBytesToSend, uxBlockTimeTicks);
      }
    }
#endif

    //		run_command_line();
  }
}

// static void vNotifierTask(void *pvParameters)
//{
//
//   // this is only used in ISR
//   // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
//   for (;;)
//   {
//     xTaskNotifyGiveIndexed(vADCTCPTaskHandle, xADCNotifyIndex);
//     xTaskNotifyGiveIndexed(vAuxADCTCPTaskHandle, xADCNotifyIndex);
//     //      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//     vTaskDelay(pdMS_TO_TICKS(30));
//   }
// }

// thanks Deepseek R1
static void vADCTCPTask(void *pvParameters)
{
  Socket_t xSocket = FREERTOS_INVALID_SOCKET;
  static const TickType_t xTimeOut = pdMS_TO_TICKS(500);
  struct freertos_sockaddr xRemoteAddress;
  BaseType_t xAlreadyTransmitted, xBytesSent;
  char *pcBufferToTransmit;
  const size_t xTotalLengthToSend = sizeof(usHSADCData0);
  uint32_t ulCurrBuf;

  /* Remote address setup */
  memset(&xRemoteAddress, 0, sizeof(xRemoteAddress));
  xRemoteAddress.sin_port = FreeRTOS_htons(usADCPort);
  xRemoteAddress.sin_address.ulIP_IPv4 = FreeRTOS_inet_addr_quick(ucServerIPAddress[0], ucServerIPAddress[1], ucServerIPAddress[2], ucServerIPAddress[3]);
  xRemoteAddress.sin_family = FREERTOS_AF_INET4;

  for (;;)
  {
    /* Wait for data notification */
    xTaskNotifyWait(0x00, 0xffffffff, &ulCurrBuf, portMAX_DELAY);
    /* Create socket and connect if not already connected */
    if (xSocket == FREERTOS_INVALID_SOCKET)
    {
      /* Create new socket */
      xSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP);
      if (xSocket == FREERTOS_INVALID_SOCKET)
      {
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait before retry
        continue;
      }

      /* Configure socket options */
      WinProperties_t xWinProperties;
      memset(&xWinProperties, '\0', sizeof xWinProperties);
      xWinProperties.lTxBufSize = ipconfigIPERF_TX_BUFSIZE;
      xWinProperties.lTxWinSize = ipconfigIPERF_TX_WINSIZE;
      xWinProperties.lRxBufSize = ipconfigIPERF_RX_BUFSIZE;
      xWinProperties.lRxWinSize = ipconfigIPERF_RX_WINSIZE;

      FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_RCVTIMEO, &xTimeOut, sizeof(xTimeOut));
      FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_SNDTIMEO, &xTimeOut, sizeof(xTimeOut));
      FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_WIN_PROPERTIES, &xWinProperties, sizeof(xWinProperties));

      /* Attempt connection */
      if (FreeRTOS_connect(xSocket, &xRemoteAddress, sizeof(xRemoteAddress)) != 0)
      {
        FreeRTOS_closesocket(xSocket);
        xSocket = FREERTOS_INVALID_SOCKET;
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait before retry
        continue;
      }
    }

    /* Select buffer based on notification */
    pcBufferToTransmit = (ulCurrBuf & 1) ? usHSADCData1 : usHSADCData0;

    /* Send data through persistent connection */
    xAlreadyTransmitted = 0;
    while (xAlreadyTransmitted < xTotalLengthToSend)
    {
      BaseType_t xAvlSpace = 0;
      BaseType_t xBytesToSend = 0;
      uint8_t *pucTCPZeroCopyStrmBuffer = FreeRTOS_get_tx_head(xSocket, &xAvlSpace);

      if (!pucTCPZeroCopyStrmBuffer)
        break;

      xBytesToSend = (xTotalLengthToSend - xAlreadyTransmitted) > xAvlSpace ? xAvlSpace : (xTotalLengthToSend - xAlreadyTransmitted);

      memcpy(pucTCPZeroCopyStrmBuffer,
             (uint8_t *)pcBufferToTransmit + xAlreadyTransmitted,
             xBytesToSend);

      xBytesSent = FreeRTOS_send(xSocket, NULL, xBytesToSend, 0);

      if (xBytesSent >= 0)
      {
        xAlreadyTransmitted += xBytesSent;
      }
      else
      {
        break; // Send error occurred
      }
    }

    /* Handle partial/failed transmission */
    if (xAlreadyTransmitted < xTotalLengthToSend)
    {
      FreeRTOS_closesocket(xSocket);
      xSocket = FREERTOS_INVALID_SOCKET;
    }
  }
}

static void vAuxADCTCPTask(void *pvParameters)
{
  Socket_t xSocket = FREERTOS_INVALID_SOCKET;
  static const TickType_t xTimeOut = pdMS_TO_TICKS(500);
  struct freertos_sockaddr xRemoteAddress;
  BaseType_t xAlreadyTransmitted, xBytesSent;
  char *pcBufferToTransmit;
  const size_t xTotalLengthToSend = sizeof(usGADCData0);
  uint32_t ulCurrBuf;

  /* Remote address setup */
  memset(&xRemoteAddress, 0, sizeof(xRemoteAddress));
  xRemoteAddress.sin_port = FreeRTOS_htons(usAuxADCPort);
  xRemoteAddress.sin_address.ulIP_IPv4 = FreeRTOS_inet_addr_quick(ucServerIPAddress[0], ucServerIPAddress[1], ucServerIPAddress[2], ucServerIPAddress[3]);
  xRemoteAddress.sin_family = FREERTOS_AF_INET4;

  for (;;)
  {
    /* Wait for data notification */
    xTaskNotifyWait(0x00, 0xffffffff, &ulCurrBuf, portMAX_DELAY);
    /* Create socket and connect if not already connected */
    if (xSocket == FREERTOS_INVALID_SOCKET)
    {
      /* Create new socket */
      xSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP);
      if (xSocket == FREERTOS_INVALID_SOCKET)
      {
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait before retry
        continue;
      }

      /* Configure socket options */
      WinProperties_t xWinProperties;
      memset(&xWinProperties, '\0', sizeof xWinProperties);
      xWinProperties.lTxBufSize = ipconfigIPERF_TX_BUFSIZE;
      xWinProperties.lTxWinSize = ipconfigIPERF_TX_WINSIZE;
      xWinProperties.lRxBufSize = ipconfigIPERF_RX_BUFSIZE;
      xWinProperties.lRxWinSize = ipconfigIPERF_RX_WINSIZE;

      FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_RCVTIMEO, &xTimeOut, sizeof(xTimeOut));
      FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_SNDTIMEO, &xTimeOut, sizeof(xTimeOut));
      FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_WIN_PROPERTIES, &xWinProperties, sizeof(xWinProperties));

      /* Attempt connection */
      if (FreeRTOS_connect(xSocket, &xRemoteAddress, sizeof(xRemoteAddress)) != 0)
      {
        FreeRTOS_closesocket(xSocket);
        xSocket = FREERTOS_INVALID_SOCKET;
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait before retry
        continue;
      }
    }

    /* Select buffer based on notification */
    pcBufferToTransmit = (ulCurrBuf & 1) ? usGADCData1 : usGADCData0;

    /* Send data through persistent connection */
    xAlreadyTransmitted = 0;
    while (xAlreadyTransmitted < xTotalLengthToSend)
    {
      BaseType_t xAvlSpace = 0;
      BaseType_t xBytesToSend = 0;
      uint8_t *pucTCPZeroCopyStrmBuffer = FreeRTOS_get_tx_head(xSocket, &xAvlSpace);

      if (!pucTCPZeroCopyStrmBuffer)
        break;

      xBytesToSend = (xTotalLengthToSend - xAlreadyTransmitted) > xAvlSpace ? xAvlSpace : (xTotalLengthToSend - xAlreadyTransmitted);

      memcpy(pucTCPZeroCopyStrmBuffer,
             (uint8_t *)pcBufferToTransmit + xAlreadyTransmitted,
             xBytesToSend);

      xBytesSent = FreeRTOS_send(xSocket, NULL, xBytesToSend, 0);

      if (xBytesSent >= 0)
      {
        xAlreadyTransmitted += xBytesSent;
      }
      else
      {
        break; // Send error occurred
      }
    }

    /* Handle partial/failed transmission */
    if (xAlreadyTransmitted < xTotalLengthToSend)
    {
      FreeRTOS_closesocket(xSocket);
      xSocket = FREERTOS_INVALID_SOCKET;
    }
  }
}

static void vTCADCTCPTask(void *pvParameters)
{
  Socket_t xSocket = FREERTOS_INVALID_SOCKET;
  static const TickType_t xTimeOut = pdMS_TO_TICKS(500);
  struct freertos_sockaddr xRemoteAddress;
  BaseType_t xAlreadyTransmitted, xBytesSent;
  char *pcBufferToTransmit;
  const size_t xTotalLengthToSend = sizeof(usTCADCData0);
  uint32_t ulCurrBuf;

  /* Remote address setup */
  memset(&xRemoteAddress, 0, sizeof(xRemoteAddress));
  xRemoteAddress.sin_port = FreeRTOS_htons(usTCADCPort);
  xRemoteAddress.sin_address.ulIP_IPv4 = FreeRTOS_inet_addr_quick(ucServerIPAddress[0], ucServerIPAddress[1], ucServerIPAddress[2], ucServerIPAddress[3]);
  xRemoteAddress.sin_family = FREERTOS_AF_INET4;

  for (;;)
  {

    /* Wait for data notification */
    xTaskNotifyWait(0x00, 0xffffffff, &ulCurrBuf, portMAX_DELAY);
    /* Create socket and connect if not already connected */
    if (xSocket == FREERTOS_INVALID_SOCKET)
    {
      /* Create new socket */
      xSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP);
      if (xSocket == FREERTOS_INVALID_SOCKET)
      {
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait before retry
        continue;
      }

      /* Configure socket options */
      WinProperties_t xWinProperties;
      memset(&xWinProperties, '\0', sizeof xWinProperties);
      xWinProperties.lTxBufSize = ipconfigIPERF_TX_BUFSIZE;
      xWinProperties.lTxWinSize = ipconfigIPERF_TX_WINSIZE;
      xWinProperties.lRxBufSize = ipconfigIPERF_RX_BUFSIZE;
      xWinProperties.lRxWinSize = ipconfigIPERF_RX_WINSIZE;

      FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_RCVTIMEO, &xTimeOut, sizeof(xTimeOut));
      FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_SNDTIMEO, &xTimeOut, sizeof(xTimeOut));
      FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_WIN_PROPERTIES, &xWinProperties, sizeof(xWinProperties));

      /* Attempt connection */
      if (FreeRTOS_connect(xSocket, &xRemoteAddress, sizeof(xRemoteAddress)) != 0)
      {
        FreeRTOS_closesocket(xSocket);
        xSocket = FREERTOS_INVALID_SOCKET;
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait before retry
        continue;
      }
    }

    /* Select buffer based on notification */
    pcBufferToTransmit = (ulCurrBuf & 1) ? usTCADCData1 : usTCADCData0;

    /* Send data through persistent connection */
    xAlreadyTransmitted = 0;
    while (xAlreadyTransmitted < xTotalLengthToSend)
    {
      BaseType_t xAvlSpace = 0;
      BaseType_t xBytesToSend = 0;
      uint8_t *pucTCPZeroCopyStrmBuffer = FreeRTOS_get_tx_head(xSocket, &xAvlSpace);

      if (!pucTCPZeroCopyStrmBuffer)
        break;

      xBytesToSend = (xTotalLengthToSend - xAlreadyTransmitted) > xAvlSpace ? xAvlSpace : (xTotalLengthToSend - xAlreadyTransmitted);

      memcpy(pucTCPZeroCopyStrmBuffer,
             (uint8_t *)pcBufferToTransmit + xAlreadyTransmitted,
             xBytesToSend);

      xBytesSent = FreeRTOS_send(xSocket, NULL, xBytesToSend, 0);

      if (xBytesSent >= 0)
      {
        xAlreadyTransmitted += xBytesSent;
      }
      else
      {
        break; // Send error occurred
      }
    }

    /* Handle partial/failed transmission */
    if (xAlreadyTransmitted < xTotalLengthToSend)
    {
      FreeRTOS_closesocket(xSocket);
      xSocket = FREERTOS_INVALID_SOCKET;
    }
  }
}

void vCommandServerTask(void *pvParameters)
{
  struct freertos_sockaddr xClient, xBindAddress;
  Socket_t xListeningSocket, xConnectedSocket;
  socklen_t xSize = sizeof(xClient);
  static const TickType_t xReceiveTimeOut = pdMS_TO_TICKS(500);
  const BaseType_t xBacklog = 20;

  /* Attempt to open the socket. */
  xListeningSocket = FreeRTOS_socket(FREERTOS_AF_INET4,    /* Or FREERTOS_AF_INET6 for IPv6. */
                                     FREERTOS_SOCK_STREAM, /* SOCK_STREAM for TCP. */
                                     FREERTOS_IPPROTO_TCP);

  /* Check the socket was created. */
  configASSERT(xListeningSocket != FREERTOS_INVALID_SOCKET);

  /* If FREERTOS_SO_RCVBUF or FREERTOS_SO_SNDBUF are to be used with
FreeRTOS_setsockopt() to change the buffer sizes from their default then do
it here!. (see the FreeRTOS_setsockopt() documentation. */

  /* If ipconfigUSE_TCP_WIN is set to 1 and FREERTOS_SO_WIN_PROPERTIES is to
be used with FreeRTOS_setsockopt() to change the sliding window size from
its default then do it here! (see the FreeRTOS_setsockopt()
documentation. */

  /* Set a time out so accept() will just wait for a connection. */
  FreeRTOS_setsockopt(xListeningSocket,
                      0,
                      FREERTOS_SO_RCVTIMEO,
                      &xReceiveTimeOut,
                      sizeof(xReceiveTimeOut));

  /* Set the listening port to 10000. */
  memset(&xBindAddress, 0, sizeof(xBindAddress));
  xBindAddress.sin_port = usCommandPort;
  xBindAddress.sin_port = FreeRTOS_htons(xBindAddress.sin_port);
  xBindAddress.sin_family = FREERTOS_AF_INET4; /* FREERTOS_AF_INET6 to be used for IPv6 */

  /* Bind the socket to the port that the client RTOS task will send to. */
  FreeRTOS_bind(xListeningSocket, &xBindAddress, sizeof(xBindAddress));

  /* Set the socket into a listening state so it can accept connections.
The maximum number of simultaneous connections is limited to 20. */
  FreeRTOS_listen(xListeningSocket, xBacklog);

  for (;;)
  {
    /* Wait for incoming connections. */
    xConnectedSocket = FreeRTOS_accept(xListeningSocket, &xClient, &xSize);
    configASSERT(xConnectedSocket != FREERTOS_INVALID_SOCKET);

    /* Spawn a RTOS task to handle the connection. */
    xTaskCreate(prvCommandHandlerTask,
                "CommandHandler",
                mainTCP_SERVER_STACK_SIZE,
                (void *)xConnectedSocket,
                tskIDLE_PRIORITY,
                NULL);
  }
}

static void prvCommandHandlerTask(void *pvParameters)
{
  Socket_t xSocket;
  static char cRxedData[CMD_BUFFER_SIZE];
  BaseType_t lBytesReceived;

  /* It is assumed the socket has already been created and connected before
being passed into this RTOS task using the RTOS task's parameter. */
  xSocket = (Socket_t)pvParameters;

  for (;;)
  {
    /* Receive another block of data into the cRxedData buffer. */
    lBytesReceived = FreeRTOS_recv(xSocket, &cRxedData, CMD_BUFFER_SIZE, 0);

    if (lBytesReceived > 0)
    {
      /* Data was received, process it here. */
      // prvProcessData( cRxedData, lBytesReceived );
      cRxedData[lBytesReceived] = 0; // ensure null terminated string
      if (strncmp(cRxedData, "INIT", 4) == 0)
      {
        FreeRTOS_printf(("Received Init Command\n"));
        HAL_GPIO_WritePin(DUT_GATE_SEL_GPIO_Port, DUT_GATE_SEL_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DUT_HVDC_ENABLE_GPIO_Port, DUT_HVDC_ENABLE_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DUT_VICTRL_SEL_GPIO_Port, DUT_VICTRL_SEL_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DUT_VGS_IDLE_SEL_GPIO_Port, DUT_VGS_IDLE_SEL_Pin, GPIO_PIN_RESET);
      }
      if (strncmp(cRxedData, "STOP", 4) == 0)
      {
        FreeRTOS_printf(("Received Stop Command\n"));
        HAL_GPIO_WritePin(DUT_GATE_SEL_GPIO_Port, DUT_GATE_SEL_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DUT_HVDC_ENABLE_GPIO_Port, DUT_HVDC_ENABLE_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DUT_VICTRL_SEL_GPIO_Port, DUT_VICTRL_SEL_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DUT_VGS_IDLE_SEL_GPIO_Port, DUT_VGS_IDLE_SEL_Pin, GPIO_PIN_RESET);
      }
      if (strncmp(cRxedData, "HEAT", 4) == 0)
      {
        FreeRTOS_printf(("Received Heat Command\n"));
        //        GPIOE->BSRR = DUT_GATE_SEL_Pin << 16 | DUT_VICTRL_SEL_Pin << 16 | DUT_HVDC_ENABLE_Pin;
        HAL_GPIO_WritePin(DUT_HVDC_ENABLE_GPIO_Port, DUT_HVDC_ENABLE_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DUT_VICTRL_SEL_GPIO_Port, DUT_VICTRL_SEL_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DUT_GATE_SEL_GPIO_Port, DUT_GATE_SEL_Pin, GPIO_PIN_RESET);
        ulSevenSegD1 |= 1 << 5; // turn first digit DP on
      }
      if (strncmp(cRxedData, "COOLA", 5) == 0)
      {
        FreeRTOS_printf(("Received Advanced Cool Command\n"));
        //        GPIOE->BSRR = DUT_GATE_SEL_Pin | DUT_VICTRL_SEL_Pin | DUT_HVDC_ENABLE_Pin << 16;
        // this sequence helps to get better results in terms of charge injection
        taskENTER_CRITICAL();
        HAL_GPIO_WritePin(DUT_HVDC_ENABLE_GPIO_Port, DUT_HVDC_ENABLE_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DUT_GATE_SEL_GPIO_Port, DUT_GATE_SEL_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DUT_VICTRL_SEL_GPIO_Port, DUT_VICTRL_SEL_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 5000; i++)
        {
          asm("nop");
        }
        HAL_GPIO_WritePin(DUT_HVDC_ENABLE_GPIO_Port, DUT_HVDC_ENABLE_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 5000; i++)
        {
          asm("nop");
        }
        HAL_GPIO_WritePin(DUT_HVDC_ENABLE_GPIO_Port, DUT_HVDC_ENABLE_Pin, GPIO_PIN_RESET);
        taskEXIT_CRITICAL();
        ulSevenSegD1 &= ~(1 << 5); // turn first digit DP off
      }
      if (strncmp(cRxedData, "COOLB", 5) == 0)
      {
        FreeRTOS_printf(("Received Basic Cool Command\n"));
        GPIOE->BSRR = DUT_GATE_SEL_Pin | DUT_VICTRL_SEL_Pin | DUT_HVDC_ENABLE_Pin << 16;
        ulSevenSegD1 &= ~(1 << 5); // turn first digit DP off
      }
    }
    else if (lBytesReceived == 0)
    {
      /* No data was received, but FreeRTOS\_recv() did not return an error.
   Timeout? */
    }
    else
    {
      /* Error (maybe the connected socket already shut down the socket?).
   Attempt graceful shutdown. */
      FreeRTOS_shutdown(xSocket, FREERTOS_SHUT_RDWR);
      break;
    }
  }

  /* The RTOS task will get here if an error is received on a read. Ensure the
socket has shut down (indicated by FreeRTOS\_recv() returning a -pdFREERTOS\_ERRNO\_EINVAL
error before closing the socket). */

  while (FreeRTOS_recv(xSocket, &usZero, 1, 0) >= 0)
  {
    /* Wait for shutdown to complete. If a receive block time is used then
       this delay will not be necessary as FreeRTOS\_recv() will place the RTOS task
       into the Blocked state anyway. */
    vTaskDelay(pdTICKS_TO_MS(1));

    /* Note - real applications should implement a timeout here, not just
       loop forever. */
  }

  /* Shutdown is complete and the socket can be safely closed. */
  FreeRTOS_closesocket(xSocket);

  /* Must not drop off the end of the RTOS task - delete the RTOS task. */
  vTaskDelete(NULL);
}

void vHandshakeTask(void* pvParameters) {
    Socket_t xSocket = FREERTOS_INVALID_SOCKET;
    static const TickType_t xTimeOut = pdMS_TO_TICKS(500);
    struct freertos_sockaddr xRemoteAddress;
    BaseType_t xBytesSent;
    BaseType_t xTotalBytesSent;
    char cJsonBuffer[512]; // Increased buffer size for JSON data
    int xJsonLength;

    /* Remote address setup */
    memset(&xRemoteAddress, 0, sizeof(xRemoteAddress));
    xRemoteAddress.sin_port = FreeRTOS_htons(usHandshakePort);
    xRemoteAddress.sin_address.ulIP_IPv4 = FreeRTOS_inet_addr_quick(ucServerIPAddress[0], ucServerIPAddress[1], ucServerIPAddress[2], ucServerIPAddress[3]);
    xRemoteAddress.sin_family = FREERTOS_AF_INET4;

    for (;;)
    {
        /* Wait for data notification */
        xTaskNotifyWait(0x00, 0xffffffff, NULL, portMAX_DELAY);
        FreeRTOS_printf(("Attempting Handshake...\n"));

        /* Create socket and connect if not already connected */
        if (xSocket == FREERTOS_INVALID_SOCKET)
        {
            /* Create new socket */
            xSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP);
            if (xSocket == FREERTOS_INVALID_SOCKET)
            {
                vTaskDelay(pdMS_TO_TICKS(100)); // Wait before retry
                continue;
            }

            /* Configure socket options */
            WinProperties_t xWinProperties;
            memset(&xWinProperties, '\0', sizeof xWinProperties);
            xWinProperties.lTxBufSize = ipconfigIPERF_TX_BUFSIZE;
            xWinProperties.lTxWinSize = ipconfigIPERF_TX_WINSIZE;
            xWinProperties.lRxBufSize = ipconfigIPERF_RX_BUFSIZE;
            xWinProperties.lRxWinSize = ipconfigIPERF_RX_WINSIZE;

            FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_RCVTIMEO, &xTimeOut, sizeof(xTimeOut));
            FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_SNDTIMEO, &xTimeOut, sizeof(xTimeOut));
            FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_WIN_PROPERTIES, &xWinProperties, sizeof(xWinProperties));

            /* Attempt connection */
            if (FreeRTOS_connect(xSocket, &xRemoteAddress, sizeof(xRemoteAddress)) != 0)
            {
                FreeRTOS_closesocket(xSocket);
                xSocket = FREERTOS_INVALID_SOCKET;
                vTaskDelay(pdMS_TO_TICKS(100)); // Wait before retry
                continue;
            }
        }



        /* Construct JSON data including device UUID and other parameters */
        xJsonLength = snprintf(cJsonBuffer, sizeof(cJsonBuffer),
                             "{"
                             "\"uuid\":\"%lu-%lu-%lu\","      /* Device UUID - replace with your variable */
			     "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
                             "\"firmware\":\"%s\"," /* Firmware version - replace with your variable */
			     "\"hardware\":\"%s\","
			     "\"vgsSampleRate\":\"%lu\","
			     "\"vdsSampleRate\":\"%lu\","
			     "\"tcSampleRate\":\"%lu\","

                             "}\r\n",
			     ulUID[0], ulUID[1], ulUID[2],            /* Replace with your UUID variable */
			     ucMACAddress[0], ucMACAddress[1], ucMACAddress[2], ucMACAddress[3], ucMACAddress[4], ucMACAddress[5],
			     ucFirmwareVersion,
			     ucHardwareVersion,
			     ulADCSR,
			     ulAuxADCSR,
			     ulTCADCSR);


        if (xJsonLength < 0 || xJsonLength >= sizeof(cJsonBuffer)) {
            /* JSON construction error or truncation occurred */
            FreeRTOS_closesocket(xSocket);
            xSocket = FREERTOS_INVALID_SOCKET;
            continue;
        }

        /* Send the JSON data */
        xTotalBytesSent = 0;
        while (xTotalBytesSent < xJsonLength)
        {
            xBytesSent = FreeRTOS_send(xSocket,
                                      &cJsonBuffer[xTotalBytesSent],
                                      xJsonLength - xTotalBytesSent,
                                      0);

            if (xBytesSent > 0)
            {
                xTotalBytesSent += xBytesSent;
            }
            else
            {
                /* Send error occurred */
                FreeRTOS_closesocket(xSocket);
                xSocket = FREERTOS_INVALID_SOCKET;
                break;
            }
        }
    }
}

// credit Claude
void prvGenerateMACFromUID(const uint32_t *uid_96bit, uint8_t *mac_addr)
{
  // Extract and XOR corresponding bytes

  // Handle first 4 bytes
  uint32_t first_word = uid_96bit[0];
  uint32_t third_word = uid_96bit[2];

  mac_addr[0] = ((first_word >> 24) & 0xFF) ^ ((third_word >> 24) & 0xFF);
  mac_addr[1] = ((first_word >> 16) & 0xFF) ^ ((third_word >> 16) & 0xFF);
  mac_addr[2] = ((first_word >> 8) & 0xFF) ^ ((third_word >> 8) & 0xFF);
  mac_addr[3] = (first_word & 0xFF) ^ (third_word & 0xFF);

  // Handle remaining 2 bytes
  uint32_t second_word = uid_96bit[1];
  mac_addr[4] = ((second_word >> 24) & 0xFF) ^ ((third_word >> 24) & 0xFF);
  mac_addr[5] = ((second_word >> 16) & 0xFF) ^ ((third_word >> 16) & 0xFF);

  // Ensure locally administered (bit 1 = 1) and unicast (bit 0 = 0)
  mac_addr[0] = (mac_addr[0] & 0xFC) | 0x02;
}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000000;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
