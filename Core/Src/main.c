/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"
#include "debug/log.h"
#include "SEN0306_mmwave.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
XSPI_HandleTypeDef hospi1;
SD_HandleTypeDef hsd1;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel3;
DMA_HandleTypeDef handle_GPDMA1_Channel2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef handle_GPDMA2_Channel5;
DMA_HandleTypeDef handle_GPDMA2_Channel4;
DMA_HandleTypeDef handle_GPDMA2_Channel0;
DMA_HandleTypeDef handle_GPDMA2_Channel3;
DMA_HandleTypeDef handle_GPDMA2_Channel2;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

bool g_sd_card_initialized = false;

/* --- INTERRUPT RING BUFFER --- */
#define RING_BUF_SIZE 512
uint8_t ring_buf[RING_BUF_SIZE];
volatile uint16_t head = 0; 
volatile uint16_t tail = 0; 
uint8_t rx_byte;            
/* ----------------------------- */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA2_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);

static bool sd_card_is_inserted(void)
{
  return HAL_GPIO_ReadPin(SD_CARD_DETECT_GPIO_Port, SD_CARD_DETECT_Pin) == GPIO_PIN_RESET;
}

/**
  * @brief  The application entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* Debug Cycle Counter Init */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA2_Init();
  MX_GPDMA1_Init();
  MX_OCTOSPI1_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();

/* USER CODE BEGIN 2 */
  sen0306_init(&huart1);
  debug_log_init(&huart2);

  // 1. FORCE INTERRUPTS & CLEAR MASK (The Magic Fixes)
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0); 
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  __set_BASEPRI(0); 

  // 2. COMMAND SENSOR TO HEX MODE
  uint8_t hex_mode_cmd[] = {0x55, 0xAA, 0x02, 0x00, 0x01, 0x00, 0x03};
  HAL_UART_Transmit(&huart1, hex_mode_cmd, sizeof(hex_mode_cmd), 100);
  
  // Give the sensor a moment to process the command
  for (volatile uint32_t i = 0; i < 500000; i++) {} 

  // 3. FLUSH THE HARDWARE BUFFER
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);

  // 4. START LISTENING TO THE SENSOR
  if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK) {
      Error_Handler();
  }
/* USER CODE END 2 */

  while (1) 
  {
      // 1. Non-blocking: Drain bytes from hardware buffer to parser
      sen0306_process_available_data();

      // 2. Grab the distance and print it
      static uint32_t last_printed_sample = 0;
      
      // Update the terminal every time we receive 5 new frames
      if (sen0306_ctx.samples_received > (last_printed_sample + 5)) 
      {
          last_printed_sample = sen0306_ctx.samples_received;
          
          // Get the latest fully parsed struct
          const sen0306_sample_t* radar_data = sen0306_get_latest();
          
          if (radar_data != NULL) {
              // Format the string
              char msg[64];
              sprintf(msg, "Target Distance: %u cm\r\n", radar_data->distance_cm);
              
              // Transmit directly out of the debug USB port (huart2)
              HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
          }
      }
  }
}

/* --- PERIPHERAL INITIALIZATION LOGIC --- */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

static void MX_GPDMA1_Init(void)
{
  __HAL_RCC_GPDMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);
}

static void MX_GPDMA2_Init(void)
{
  __HAL_RCC_GPDMA2_CLK_ENABLE();
  __HAL_RCC_SRAM2_CLK_ENABLE();
  HAL_NVIC_SetPriority(GPDMA2_Channel0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA2_Channel0_IRQn);
  HAL_NVIC_SetPriority(GPDMA2_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA2_Channel2_IRQn);
  HAL_NVIC_SetPriority(GPDMA2_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA2_Channel3_IRQn);
  HAL_NVIC_SetPriority(GPDMA2_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA2_Channel4_IRQn);
  HAL_NVIC_SetPriority(GPDMA2_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA2_Channel5_IRQn);
}

static void MX_OCTOSPI1_Init(void)
{
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThresholdByte = 1;
  hospi1.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
  hospi1.Init.MemoryType = HAL_XSPI_MEMTYPE_MICRON;
  hospi1.Init.MemorySize = HAL_XSPI_SIZE_128MB;
  hospi1.Init.ChipSelectHighTimeCycle = 1;
  hospi1.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 0;
  hospi1.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
  hospi1.Init.DelayBlockBypass = HAL_XSPI_DELAY_BLOCK_BYPASS;
  hospi1.Init.Refresh = 0;
  if (HAL_XSPI_Init(&hospi1) != HAL_OK) { Error_Handler(); }
}

static void MX_SDMMC1_SD_Init(void)
{
  g_sd_card_initialized = false;
  if (!sd_card_is_inserted()) { return; }
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 4;
  if (HAL_SD_Init(&hsd1) != HAL_OK) { return; }
  g_sd_card_initialized = true;
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) { Error_Handler(); }
}

static void MX_SPI4_Init(void)
{
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x7;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi4.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi4.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi4) != HAL_OK) { Error_Handler(); }
}

static void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK) { Error_Handler(); }
  HAL_UARTEx_DisableFifoMode(&huart4);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
  HAL_UARTEx_DisableFifoMode(&huart1);

  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0); 
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
  HAL_UARTEx_DisableFifoMode(&huart2);
}

static void MX_USB_PCD_Init(void)
{
#ifdef ULYSSES_USE_USB
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK) { Error_Handler(); }
#endif
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, BNO_RST_Pin|BNO_CS_Pin|EXT_CS_2_Pin|EXT_CS_3_Pin|EXT_CS_4_Pin|BARO2_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, BNO_WAKE_Pin|BNO_BOOTN_Pin|STAT_LED_R_Pin|STAT_LED_G_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, EXT_CS_1_Pin|BMI_GYRO_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, BARO1_CS_Pin|BMI_ACC_CS_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = BNO_RST_Pin|BNO_CS_Pin|EXT_CS_2_Pin|EXT_CS_3_Pin|EXT_CS_4_Pin|BARO2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BNO_WAKE_Pin|BNO_BOOTN_Pin|STAT_LED_R_Pin|STAT_LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) { HAL_IncTick(); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

/* USER CODE BEGIN 4 */

volatile uint32_t hardware_errors = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint16_t next = (head + 1) % RING_BUF_SIZE;
    if (next != tail) {
        ring_buf[head] = rx_byte;
        head = next;
    }
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    hardware_errors++; // COUNT THE ERRORS
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    HAL_UART_Receive_IT(huart, &rx_byte, 1); // Force resume
  }
}
/* USER CODE END 4 */