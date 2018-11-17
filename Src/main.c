
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "serial_io.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef enum
{
    IDLE,
    I2C_DATA_READY,
    WAITING_FOR_HEADER,
    HEADER_RECEIVED,
    WAITING_FOR_PAYLOAD,
    PAYLOAD_RECEIVED,
    WAITING_FRAME_SEND,
} I2C_RX_STATE;

typedef enum
{
    UART_IDLE,
    UART_WAITING_FOR_HEADER,
    UART_HEADER_READY,
    UART_WAITING_FOR_FRAME,
    UART_FRAME_READY,
    UART_WAITING_FOR_DISPATCHING,
    UART_MEASSAGE_DISPATCHED,
} UART_RX_STATE;



typedef struct i2c_heder_t_
{
    uint8_t size;
    uint8_t flags;
    uint8_t seqnr;
    uint8_t msgid;
} i2c_heder_t;

static const uint8_t UART_HEADER[]={0xFE, 0xFF};
static const uint32_t RX_TIMEOUT = 1000;
static const uint32_t TEMP_LOG_TIMEOUT_ms = 1000;
static const uint16_t I2C_MSG_SIZE = BUF_SIZE;


static uint8_t i2c_buf[BUF_SIZE];
static uint8_t rx_buf[BUF_SIZE];
static uint8_t tx_buf[BUF_SIZE];
static GPIO_InitTypeDef skywr_pin;
static sio_t log_io = {
    .uart = &huart1,
    .ready = {true, true},
    .buffer_size ={BUF_SIZE, BUF_SIZE},
    .bytes_in_buffer = {0,0},
    .buffer = {rx_buf, tx_buf}
    //.buffer = {NULL, NULL}
};
static I2C_RX_STATE i2c_state = IDLE;
#if 0
// Size  Flags  Seq   ID      Command        Reserved      Argument 0                     Argument 1
// Command, Argument 1 and Argument 2 are sent LSByte first, so 1 = 0x01 0x00 etc
// Enable AirWheel, requires 0x20 to be sent in Argument 0 and 1
static const uint8_t air_wheel_en[] = {0x00, 0x00, 0xA2, 0x90, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00};
// Enable all gestures and X/Y/Z data, 0 = Garbage, 1 = Flick WE, 2 = Flick EW, 3 = Flick SN, 4 = Flick NS, 5 = Circle CW, 6 = Circle CCW
static const uint8_t geasture_en[] = {0x00, 0x00, 0xA2, 0x85, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00};
// Enable all data output 0 = DSP, 1 = Gesture, 2 = Touch, 3 = AirWheel, 4 = Position
static const uint8_t all_en[] = {0x00, 0x00, 0xA2, 0xA0, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00};
// Disable auto-calibration
static const uint8_t autocalib_disable[] = {0x00, 0x00, 0xA2, 0x80, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00};
#endif
/*
 * UART RX stuff
 */
static UART_RX_STATE uart_state = UART_IDLE;
/*
 * Calibration values for ADC
 */
static const uint16_t* TS_CAL1=(uint16_t*)(0x1FFF75A8);
static const uint16_t* TS_CAL2=(uint16_t*)(0x1FFF75CA);
static const float TS_T1_C = TEMPSENSOR_CAL1_TEMP;
static const float TS_T2_C = TEMPSENSOR_CAL2_TEMP;
static const uint16_t* VREFINT=(uint16_t*)(0x1FFF75AA);
static const float CALIB_VOLTAGE_mV = VREFINT_CAL_VREF;
static const float VDDA_mV = 3300.0;
static const uint32_t analog_in_map[ANALOG_IN_CNT] =
{
    ADC_CHANNEL_5,
    ADC_CHANNEL_VREFINT,
    ADC_CHANNEL_TEMPSENSOR,
    ADC_CHANNEL_VBAT
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*
 * Handling of communication to the I2C
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ( (GPIO_Pin == Skywriter_TRFR_Pin) && (i2c_state == IDLE))
    {
        HAL_GPIO_WritePin(DIAG_OUT_GPIO_Port, DIAG_OUT_Pin, GPIO_PIN_SET);
        i2c_state = I2C_DATA_READY;
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    printf("HAL_I2C_MemRxCpltCallback"NL);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    if ((I2cHandle == &hi2c1) && (i2c_state == WAITING_FOR_HEADER))
    {
        i2c_state = HEADER_RECEIVED;
    }
    if ((I2cHandle == &hi2c1) && (i2c_state == WAITING_FOR_PAYLOAD))
    {
        i2c_state = PAYLOAD_RECEIVED;
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    if ((I2cHandle == &hi2c1) && (uart_state == UART_WAITING_FOR_DISPATCHING))
    {
        uart_state = UART_MEASSAGE_DISPATCHED;
        i2c_state = IDLE;
    }
}
/*
 * Handle of UART RX to the host
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == USART1)
    {
        if (uart_state == UART_WAITING_FOR_HEADER)
        {
            uart_state = UART_HEADER_READY;
        }
        else if (uart_state == UART_WAITING_FOR_FRAME)
        {
            uart_state = UART_FRAME_READY;
        }
    }
}
/*
 * UART state machine for data received from the host
 */
void handle_uart_rx(UART_HandleTypeDef *uart)
{
    if (uart_state == UART_IDLE)
    {
        HAL_UART_Receive_DMA(uart, rx_buf, UART_HEADER_SIZE+I2C_HEADER_SIZE);
        uart_state = UART_WAITING_FOR_HEADER;
    }
    else if (uart_state == UART_HEADER_READY)
    {
        uint16_t msg_size = rx_buf[UART_HEADER_SIZE];
        HAL_UART_Receive_DMA(uart, &rx_buf[UART_HEADER_SIZE+I2C_HEADER_SIZE], msg_size);
        uart_state = UART_WAITING_FOR_DISPATCHING;
    }
    else if (uart_state == UART_FRAME_READY)
    {
        uart_state = UART_WAITING_FOR_DISPATCHING;
    }
    else if (uart_state == UART_MEASSAGE_DISPATCHED)
    {
        HAL_UART_Receive_DMA(uart, rx_buf, UART_HEADER_SIZE+I2C_HEADER_SIZE);
        uart_state = UART_WAITING_FOR_HEADER;
    }
    return;
}

/*
 * ADC(inkl. Temperature)/DAC Stuff
 */
static int16_t adc_read_value(ADC_CH adc_ch)
{
    int32_t raw_value = -1;
    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel = analog_in_map[adc_ch];
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        printf("Confiuration failed"NL);
        return raw_value;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      /* Start Conversation Error */
      printf("Start conversion error"NL);
      return raw_value;
    }

    /*##-4- Wait for the end of conversion #####################################*/
    /*  For simplicity reasons, this example is just waiting till the end of the
        conversion, but application may perform other tasks while conversion
        operation is ongoing. */
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
    {
      /* End Of Conversion flag not set on time */
        printf("End Of Conversion error"NL);
        return raw_value;
    }
    else
    {
        raw_value = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    return raw_value;
}


float read_temperature()
{
    float temp=-300.0;
    uint32_t raw_value = adc_read_value(TEMPERATURE);

    if (raw_value >= 0)
    {
        int32_t cal1 = *TS_CAL1;
        int32_t cal2 = *TS_CAL2;
        float delta_value = cal2-cal1;
        float ref_v_ratio = VDDA_mV/CALIB_VOLTAGE_mV;
        float dif_val_corr;
        /* ADC conversion completed */
        /*##-5- Get the converted value of regular channel  ########################*/
        dif_val_corr = ref_v_ratio*raw_value-cal1;
        //printf("cal1/cal2/read = %d/%d/%d"NL, cal1, cal2, raw_value);
        temp = (TS_T2_C-TS_T1_C)*dif_val_corr/delta_value+TS_T1_C;
    }
    return temp;
}

float read_voltage_mV(ADC_CH ADC_IN)
{
    float voltage_mV = -1;
    uint32_t raw_value = adc_read_value(ADC_IN);

    if (raw_value >= 0)
    {
        uint8_t res_bit = 12-2*(hadc1.Init.Resolution>>ADC_CFGR_RES_Pos);
        voltage_mV = raw_value;
        voltage_mV *= VDDA_mV;
        voltage_mV/= (1<<res_bit);
    }
    return voltage_mV;
}

int16_t write_value(uint16_t value)
{
    /*##-3- Set DAC Channel1 DHR register ######################################*/
    if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value) != HAL_OK)
    {
        /* Setting value Error */
        return -1;
    }

    /*##-4- Enable DAC Channel1 ################################################*/
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
    {
        /* Start Error */
        return -1;
    }
    return value;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint16_t timeout = 0;
    uint32_t ltick = HAL_GetTick();
    uint32_t dac_value=0;
    uint32_t dac_val_increase = 124;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    i2c_buf[0] = UART_HEADER[0];
    i2c_buf[1] = UART_HEADER[1];
    serial_io_init(&log_io);
    //printf("Start up"NEWLINE);
    skywr_pin.Pin = Skywriter_TRFR_Pin;
    skywr_pin.Mode = GPIO_MODE_IT_FALLING;
    skywr_pin.Pull = GPIO_NOPULL;
    skywr_pin.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_WritePin(Skywritter_RESET_GPIO_Port, Skywritter_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    //HAL_GPIO_WritePin(Skywritter_RESET_GPIO_Port, Skywritter_RESET_Pin, GPIO_PIN_SET);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    write_value(dac_value);
    float temp = read_temperature();
    //printf("Temperature is %f C"NL, temp);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        uint32_t tick = HAL_GetTick();
        if (i2c_state == I2C_DATA_READY)
        {
            skywr_pin.Mode = GPIO_MODE_OUTPUT_PP;
            HAL_GPIO_WritePin(GPIOB, Skywriter_TRFR_Pin, GPIO_PIN_RESET);
            HAL_GPIO_Init(GPIOB, &skywr_pin);
            timeout = 0;
            //printf("I2C data ready"NL);
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            /* obtain data from I2C */
            HAL_I2C_Master_Sequential_Receive_DMA(&hi2c1, SKYWRITER_I2C_ADR<<1, &i2c_buf[UART_HEADER_SIZE], I2C_HEADER_SIZE, I2C_FIRST_AND_NEXT_FRAME);
            i2c_state =WAITING_FOR_HEADER;
        }
        else if (i2c_state == HEADER_RECEIVED)
        {
            uint16_t msg_size = i2c_buf[UART_HEADER_SIZE]-I2C_HEADER_SIZE;
            HAL_I2C_Master_Sequential_Receive_DMA(&hi2c1, SKYWRITER_I2C_ADR<<1, &i2c_buf[UART_HEADER_SIZE+I2C_HEADER_SIZE], msg_size, I2C_LAST_FRAME);
            i2c_state =WAITING_FOR_PAYLOAD;
        }
        else if (i2c_state == PAYLOAD_RECEIVED)
        {
            i2c_buf[0] =
            _write(NULL, i2c_buf, i2c_buf[UART_HEADER_SIZE]+2);
            /*
            printf("Received I2C data (%d)"NL, timeout);
            for (uint8_t i=0;i<i2c_buf[0];i+=16)
            {
                printf("0x%02x ", i);
                for (uint8_t j=0;j<16;j++)
                {
                    printf("%02x", i2c_buf[i+j]);
                    if (j==8)
                    {
                        printf("  ");
                    }
                }
                printf(NL);
            }
            */
            HAL_GPIO_WritePin(DIAG_OUT_GPIO_Port, DIAG_OUT_Pin, GPIO_PIN_RESET);
            skywr_pin.Mode = GPIO_MODE_IT_FALLING;
            skywr_pin.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOB, &skywr_pin);
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            i2c_state = IDLE;
        }
        handle_uart_rx(&huart1);
        if ( (i2c_state == IDLE) && (uart_state == UART_WAITING_FOR_DISPATCHING))
        {
            uint16_t msg_size = rx_buf[UART_HEADER_SIZE];
            HAL_I2C_Master_Transmit_DMA(&hi2c1, SKYWRITER_I2C_ADR<<1, &rx_buf[UART_HEADER_SIZE], msg_size);
            i2c_state == WAITING_FRAME_SEND;
       }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
        ++timeout;
    }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enables the Clock Security System 
    */
  HAL_RCCEx_EnableLSECSS();

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration 
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC1 init function */
static void MX_DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00702991;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Skywritter_RESET_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIAG_OUT_GPIO_Port, DIAG_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA5 
                           PA6 PA7 PA11 PA12 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Skywritter_RESET_Pin LED_Pin */
  GPIO_InitStruct.Pin = Skywritter_RESET_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Skywriter_TRFR_Pin */
  GPIO_InitStruct.Pin = Skywriter_TRFR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Skywriter_TRFR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIAG_OUT_Pin */
  GPIO_InitStruct.Pin = DIAG_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIAG_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
