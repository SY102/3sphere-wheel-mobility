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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24.h"
#include "NRF24_conf.h"
#include "string.h"
#include "stdio.h"
#include "NRF24_reg_addresses.h"
#define PAYLOAD_SIZE 2
#define EXPECTED_CH 40
#define CH 40

extern const uint8_t NRF24_ADDR[5];
void debug_check_channel(void);
void debug_dump_settings(void);   // ← 이 줄을 추가!

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void nrf24_receiver_setup(void)
{

	  // 1) PWR-DOWN 상태에서 시작
	uint8_t rx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // 수신기 주소

	ce_low();
	    nrf24_pwr_up();
	    HAL_Delay(5);

	    // CONFIG 설정
	    uint8_t cfg = nrf24_r_reg(CONFIG,1);
	    cfg |= (1<<PWR_UP)|(1<<PRIM_RX);
	    nrf24_w_reg(CONFIG,&cfg,1);
	    HAL_Delay(1);

	    // 채널
	    nrf24_w_reg(RF_CH,    (uint8_t[]){40}, 1);

	    // 파이프0 활성화 + ACK
	    nrf24_w_reg(EN_RXADDR,(uint8_t[]){0x01},1);
	    nrf24_auto_ack(0, enable);

	    nrf24_w_reg(RX_PW_P0, (uint8_t[]){2}, 1);   //


	        nrf24_open_rx_pipe(0, rx_address);
	    // raw 읽기



	       // 6) 전원 올리고 듣기 모드 진입

	     //  printf("Before listen: CONFIG=0x%02X (PRIM_RX=%u)\r\n",
	       //       nrf24_r_reg(CONFIG,1), nrf24_r_reg(CONFIG,1)&1);

	      // printf(" After listen: CONFIG=0x%02X (PRIM_RX=%u)\r\n",
	        //      nrf24_r_reg(CONFIG,1), nrf24_r_reg(CONFIG,1)&1);

	       debug_check_channel();       // RAW RF_CH = 40 …
	           debug_dump_settings();       // RF_CH = 40, RX_ADDR_P0 = E7…
	           ce_high();

	        }

void debug_check_channel(void) {
	uint8_t cmd = R_REGISTER | RF_CH;
	    uint8_t buf[2] = { cmd, NOP_CMD };   // NOP_CMD = 0xFF
	    uint8_t rx[2]  = { 0 };

	    csn_low();
	      // 한 번에 두 바이트 교환
	      HAL_SPI_TransmitReceive(&hspi1, buf, rx, 2, spi_rw_timeout);
	    csn_high();

	    // rx[0] 에는 STATUS, rx[1] 에는 RF_CH 레지스터 값
	    printf("RAW RF_CH = %u (STATUS=0x%02X)\n", rx[1], rx[0]);
}
void debug_dump_settings(void) {
    uint8_t ch   = nrf24_r_reg(RF_CH,      1);
    uint8_t pw   = nrf24_r_reg(RX_PW_P0,   1);
    uint8_t addr[5] = {0};

    // *정확히* RX_ADDR_P0 레지스터(0x0A)부터 5바이트 읽으려면:
    csn_low();
      uint8_t cmd = R_REGISTER | RX_ADDR_P0;
      HAL_SPI_Transmit(&hspi1, &cmd, 1, spi_w_timeout);
      HAL_SPI_Receive (&hspi1, addr, 5, spi_r_timeout);
    csn_high();

    printf("=== DEBUG SETTINGS ===\r\n");
    printf(" RF_CH       = %u\r\n", ch);
    printf(" RX_PW_P0    = %u bytes\r\n", pw);
    printf(" RX_ADDR_P0  = %02X %02X %02X %02X %02X\r\n",
           addr[0], addr[1], addr[2], addr[3], addr[4]);
    printf("======================\r\n");
}
void receive_data(void)
{

	  if (!nrf24_data_available()) {
	        return;
	    }

	    // 2바이트 읽고 문자열 종료
	    uint8_t buf[PAYLOAD_SIZE+1] = {0};
	    nrf24_receive(buf, PAYLOAD_SIZE);
	    buf[PAYLOAD_SIZE] = '\0';

	    // 출력
	    printf("Received: %s\r\n", buf);



            // 수신된 데이터가 "hi"와 일치하는지 확인
            if (strcmp((char*)buf, "hi") == 0) {
                // 수신된 데이터가 "hi"이면 PC8 LED 켜기
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);  // PC8 LED 켜기
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);  // PC6 LED 끄기
                printf("성공");
            } else {
                // 수신된 데이터가 "hi"가 아니면 PC6 LED 켜기
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);  // PC6 LED 켜기
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  // PC8 LED 끄기
                printf("실패");
            }

            nrf24_clear_rx_dr();
              nrf24_flush_rx();

}


int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);  // UART2로 데이터 전송
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
 nrf24_receiver_setup();

   // 결과 출력



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  receive_data();  // 데이터 수신 시도
	 	  if (nrf24_data_available())
	 	       receive_data();       // 실제로 "hi" 를 수신하면 LED 켜고 printf
	 	     // }
	 	  //debug_check_channel();
	 	          HAL_Delay(100);  // 짧은 지연
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
