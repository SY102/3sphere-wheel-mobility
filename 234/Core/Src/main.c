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
#include "stdio.h"
#include "NRF24_reg_addresses.h"


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
uint8_t tx_data[] = {'h', 'i','i'};      // '\0' 빼고 2바이트


void nrf24_transmitter_setup(void)
{




    uint8_t tx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // 송신기 주소
    nrf24_open_tx_pipe(tx_address);

    nrf24_defaults();  // NRF24 모듈을 기본 설정으로 초기화
    nrf24_pwr_up();
    HAL_Delay(5);        // datasheet 권장: >4.5ms
    nrf24_stop_listen();


    nrf24_set_channel(40);
    nrf24_set_payload_size(2);
     nrf24_auto_ack_all(disable);
          // 내부적으로 CONFIG.PRIM_RX=0, CE=0 처리



     nrf24_open_tx_pipe((uint8_t*)tx_address);

    nrf24_tx_pwr(3); // 전송 파워 설정 (필요에 따라 조정)
    nrf24_data_rate(_1mbps); // 데이터 전송 속도 설정 (1mbps 일반적으로 사용)


}

void debug_dump_settings(void) {
    uint8_t ch     = nrf24_r_reg(RF_CH,      1);
    uint8_t pw     = nrf24_r_reg(RX_PW_P0,   1);
    uint8_t addr[5] = {0};

    // RX_ADDR_P0 레지스터(0x0A)부터 5바이트 읽기
    csn_low();
      uint8_t cmd = R_REGISTER | RX_ADDR_P0;
      HAL_SPI_Transmit(&hspiX, &cmd, 1, spi_rw_timeout);
      HAL_SPI_Receive (&hspiX, addr, 5, spi_r_timeout);
    csn_high();

    printf("=== DEBUG SETTINGS ===\r\n");
    printf(" RF_CH       = %u\r\n", ch);
    printf(" RX_PW_P0    = %u bytes\r\n", pw);
    printf(" RX_ADDR_P0  = %02X %02X %02X %02X %02X\r\n",
           addr[0], addr[1], addr[2], addr[3], addr[4]);
    printf("======================\r\n");
}

void transmit_hi(void)
{
	 uint8_t wr = 40;                    // 쓰고 싶은 채널 값
		          nrf24_w_reg(RF_CH, &wr, 1);         // RF_CH 레지스터에 40(0x28) 쓰기
		          uint8_t rd = nrf24_r_reg(RF_CH, 1); // 바로 다시 읽기
		          printf("TEST RF_CH = %u\r\n", rd);  // rd가 40인지 확인
    // 데이터 전송
    if (nrf24_transmit(tx_data, sizeof(tx_data)) == 0) {
        // 전송 성공 시, LED를 켜서 피드백
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);  // PC6 핀에 연결된 LED 켜기
        nrf24_flush_tx();  // 송신 버퍼 플러시

        printf("Sent Data: %s\n", tx_data);  // 송신된 데이터를 터미널에 출력
    } else {
        // 전송 실패 시, LED 끄기
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);  // PC6 핀에 전압을 출력 (LED 켜짐)
    	 nrf24_flush_tx();  // 송신 버퍼 플러시
    	 printf("Failed to send data: %s\n", tx_data);  // 전송 실패 메시지

    }



}

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);  // UART2로 데이터 전송
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
  nrf24_transmitter_setup();
  debug_dump_settings();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	   transmit_hi();  // "hi" 전송
	 // debug_setup();

	         HAL_Delay(1000); // 2초 대기
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
