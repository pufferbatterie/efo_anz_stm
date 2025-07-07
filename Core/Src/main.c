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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include "tiny-json.h"
#include <string.h>

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

// 0 = volt
// 1 = current
// 2 = 1V5
// 3 = 3V3
// 4 = int temp
// 5 = int vref
uint16_t adcval[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void blink_and_text() {
	HAL_GPIO_WritePin(LED_RIGHT_3_GPIO_Port, LED_RIGHT_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_LEFT_1_GPIO_Port, LED_LEFT_1_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	lcd_display_write_line(0, "   ISAE EFO Anz   ");
	lcd_display_write_line(1, "                  ");
	lcd_display_write_line(2, "                  ");
	lcd_display_write_line(3, "                  ");
	HAL_GPIO_WritePin(LED_LEFT_1_GPIO_Port, LED_LEFT_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_LEFT_2_GPIO_Port, LED_LEFT_2_Pin, GPIO_PIN_SET);
	HAL_Delay(400);
	lcd_display_write_line(0, "                  ");
	lcd_display_write_line(1, "   ISAE EFO Anz   ");
	lcd_display_write_line(2, "                  ");
	lcd_display_write_line(3, "                  ");
	HAL_GPIO_WritePin(LED_LEFT_2_GPIO_Port, LED_LEFT_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_LEFT_3_GPIO_Port, LED_LEFT_3_Pin, GPIO_PIN_SET);
	HAL_Delay(400);
	lcd_display_write_line(0, "                  ");
	lcd_display_write_line(1, "                  ");
	lcd_display_write_line(2, "   ISAE EFO Anz   ");
	lcd_display_write_line(3, "                  ");
	HAL_GPIO_WritePin(LED_LEFT_3_GPIO_Port, LED_LEFT_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_RIGHT_1_GPIO_Port, LED_RIGHT_1_Pin, GPIO_PIN_SET);
	HAL_Delay(400);
	lcd_display_write_line(0, "                  ");
	lcd_display_write_line(1, "                  ");
	lcd_display_write_line(2, "                  ");
	lcd_display_write_line(3, "   ISAE EFO Anz   ");
	HAL_GPIO_WritePin(LED_RIGHT_1_GPIO_Port, LED_RIGHT_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_RIGHT_2_GPIO_Port, LED_RIGHT_2_Pin, GPIO_PIN_SET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(LED_RIGHT_2_GPIO_Port, LED_RIGHT_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_RIGHT_3_GPIO_Port, LED_RIGHT_3_Pin, GPIO_PIN_SET);
	HAL_Delay(400);
}

#define MODE_COMMAND                              0x00
#define MODE_DATA                                 0x40

#define COMMAND_CLEAR_DISPLAY                     0x01
#define COMMAND_RETURN_HOME                       0x02
#define COMMAND_ENTRY_MODE_SET                    0x04
#define ENTRY_MODE_LEFT_TO_RIGHT                  0x02
#define ENTRY_MODE_SHIFT_INCREMENT                0x01
#define COMMAND_SHIFT                             0x10
#define COMMAND_DISPLAY_SHIFT_LEFT                0x08
#define COMMAND_DISPLAY_SHIFT_RIGHT               0x0C
#define COMMAND_CURSOR_SHIFT_LEFT                 0x00
#define COMMAND_CURSOR_SHIFT_RIGHT                0x04

#define ADDRESS_CGRAM                             0x40
#define ADDRESS_DDRAM                             0x80
#define ADDRESS_DDRAM_DOGS164_TOP_OFFSET          0x04
#define ADDRESS_DDRAM_DOGS104_TOP_OFFSET          0x0A

#define COMMAND_8BIT_4LINES_RE1_IS0               0x3A
#define COMMAND_8BIT_4LINES_RE0_IS0_DH1           0x3C
#define COMMAND_8BIT_4LINES_RE0_IS1               0x39
#define COMMAND_8BIT_4LINES_RE0_IS1_DH1           0x3D
#define COMMAND_8BIT_4LINES_RE0_IS0               0x38

// Command from extended set (RE = 1, IS = 0)
#define COMMAND_BS1_1                             0x1E
#define COMMAND_POWER_DOWN_DISABLE                0x02
#define COMMAND_TOP_VIEW                          0x05
#define COMMAND_BOTTOM_VIEW                       0x06
#define COMMAND_4LINES                            0x09
#define COMMAND_3LINES_TOP                        0x1F
#define COMMAND_3LINES_MIDDLE                     0x17
#define COMMAND_3LINES_BOTTOM                     0x13
#define COMMAND_2LINES                            0x1B

// Command from extended set (RE = 0, IS = 1)
#define COMMAND_DISPLAY                           0x08
#define COMMAND_DISPLAY_ON                        0x04
#define COMMAND_DISPLAY_OFF                       0x00
#define COMMAND_CURSOR_ON                         0x02
#define COMMAND_CURSOR_OFF                        0x00
#define COMMAND_BLINK_ON                          0x01
#define COMMAND_BLINK_OFF                         0x00

// Command from extended set (RE = 1, IS = 1)
#define COMMAND_SHIFT_SCROLL_ENABLE               0x10
#define COMMAND_SHIFT_SCROLL_ALL_LINES            0x0F
#define COMMAND_SHIFT_SCROLL_LINE_1               0x01
#define COMMAND_SHIFT_SCROLL_LINE_2               0x02
#define COMMAND_SHIFT_SCROLL_LINE_3               0x04
#define COMMAND_SHIFT_SCROLL_LINE_4               0x08

#define COMMAND_BS0_1                             0x1B
#define COMMAND_INTERNAL_DIVIDER                  0x13
#define COMMAND_CONTRAST_DEFAULT_DOGM204          0x72
#define COMMAND_CONTRAST_DEFAULT_DOGS164          0x6B
#define COMMAND_CONTRAST_DEFAULT_DOGS104          0x7A
#define COMMAND_POWER_CONTROL_DOGM204             0x57
#define COMMAND_POWER_CONTROL_DOGS164             0x56
#define COMMAND_POWER_CONTROL_DOGS104             0x56
#define COMMAND_POWER_ICON_CONTRAST               0x5C
#define COMMAND_FOLLOWER_CONTROL_DOGM204          0x6E
#define COMMAND_FOLLOWER_CONTROL_DOGS164          0x6C
#define COMMAND_FOLLOWER_CONTROL_DOGS104          0x6E
#define COMMAND_ROM_SELECT                        0x72
#define COMMAND_ROM_A                             0x00
#define COMMAND_ROM_B                             0x04
#define COMMAND_ROM_C                             0x08

#define LCD_I2C_ADDR 0x78 // 0x3C ///< The peripheral address as defined with SA0 pulled low.
#define LCD_I2C_RATE_HZ 400000UL
#define LCD_REFRESH_HZ 10U
#define LCD_ORIENT_TOP 0x05
#define LCD_ORIENT_BTM 0x06
#define LCD_ORIENT LCD_ORIENT_BTM
#define LCD_CONTRAST_MAX 0x3F
#define LCD_CONTRAST 0x0F
#define LCD_CURSOR 1
#define LCD_BLINK 1
#define LCD_N_ROW 4U  ///< Number of available rows on LCD
#define LCD_N_COL 16U ///< Number of available columns on LCD

void lcd_init() {
	HAL_GPIO_WritePin(LCD_NRESET_GPIO_Port, LCD_NRESET_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(LCD_NRESET_GPIO_Port, LCD_NRESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(4);
	HAL_GPIO_WritePin(LCD_NRESET_GPIO_Port, LCD_NRESET_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

}

inline static void i2c_write_cmd(uint8_t cmd) {
	uint8_t const buffer[] = { 0x00, cmd };
	//i2c_write_blocking(LCD_I2C_BUS, LCD_I2C_ADDR, (uint8_t const *)&buffer, sizeof(buffer), pdFALSE);
	HAL_I2C_Master_Transmit(&hi2c2, LCD_I2C_ADDR, (uint8_t*) &buffer,
			sizeof(buffer), 1000); //Sending in Blocking mode
}

inline static void i2c_write_line(uint8_t row, char const line[LCD_N_COL]) {
	uint8_t c = 0, line_buffer_index = 0U;
	uint8_t line_buffer[LCD_N_COL + sizeof(uint8_t)];
	line_buffer[line_buffer_index++] = 0x40;
	while ((line[c] != '\0') && (c < LCD_N_COL)) {
		line_buffer[line_buffer_index++] = line[c];
		c++;
	}
	while (c < LCD_N_COL) {
		line_buffer[line_buffer_index++] = ' ';
		c++;
	}
	uint8_t cmd = 0x80 + (0x20 * row);
#if LCD_ORIENT == LCD_ORIENT_TOP
    cmd += 0x04;
#endif
	i2c_write_cmd(cmd); ///> Set DDRAM address to the start of this row
	//i2c_write_blocking(LCD_I2C_BUS, LCD_I2C_ADDR, (uint8_t const *)line_buffer, line_buffer_index, pdFALSE);
	HAL_I2C_Master_Transmit(&hi2c2, LCD_I2C_ADDR, (uint8_t*) &line_buffer,
			line_buffer_index, 1000); //Sending in Blocking mode
}

void lcd_display_screen(bool enable) {
	uint8_t cmd = 0x08;
	cmd |= (enable << 2);
	cmd |= (LCD_CURSOR << 1);
	cmd |= (LCD_BLINK << 1);
	i2c_write_cmd(cmd); ///> Display on, cursor on, blink on}
}

void lcd_display_write_line(uint8_t row, char const ptext[LCD_N_COL]) {
	if (row < LCD_N_ROW) {
		i2c_write_line(row, ptext);
	}
}

void lcd_display_configure(void) {
	/* The contrast is spread over two seperate commands and so split here */
	uint8_t contrast_msb = ((LCD_CONTRAST & 0x30U) >> 4U);
	uint8_t contrast_lsb = ((LCD_CONTRAST & 0x0FU) >> 0U);
	i2c_write_cmd(0x3A); ///> Function Set, 8 bit data length extension Bit RE=1; REV=0
	i2c_write_cmd(0x09);              ///> Extended function set, 4 line display
	i2c_write_cmd(LCD_ORIENT); ///> Entry mode set, Top view, note 6 for botton view.
	i2c_write_cmd(0x1E);                ///> Bias setting, BS1=1
	i2c_write_cmd(0x39); ///> Function Set, 8 bit data length extension Bit RE=0; IS=1
	i2c_write_cmd(0x1B);                ///> Internal OSC, BS0=1 -> Bias=1/6
	i2c_write_cmd(0x6C);         ///> Follower control, Divider on and set value
	i2c_write_cmd(0x54 | contrast_msb); ///> Power control, Booster on and set contrast (DB1=C5, DB0=C4)
	i2c_write_cmd(0x70 | contrast_lsb); ///> Set contrast (DB3-DB0=C3-C0)
	i2c_write_cmd(0x38);         ///> 8 bit data length extension Bit RE=0; IS=0
	lcd_display_screen(true); ///> Display on, cursor & blink as per definitions.
	i2c_write_cmd(0x01);                ///> Clear Screen
}

void lcd_display_init_task() {
	/* Create task to keep the LCD updated when new data arrives */
	lcd_init();     ///> Toggle reset line to reset LCD controller.
	lcd_display_configure(); ///> Configure LCD commands.
	/* Add version string to the open display screen
	char linebuffer[LCD_N_COL];
	lcd_display_write_line(0, "   ISAE EFO Anz   ");
	lcd_display_write_line(1, "                  ");
	lcd_display_write_line(3, "                  ");
	*/
}

void sendCommand(uint8_t cmd) {
	uint8_t const buffer[] = { MODE_COMMAND, cmd };
	HAL_StatusTypeDef a = HAL_I2C_Master_Transmit(&hi2c2, LCD_I2C_ADDR,
			(uint8_t*) &buffer, sizeof(buffer), 1000); //Sending in Blocking mode
	HAL_Delay(1);
}

void loop_print_screen() {
	while (1) {
		HAL_GPIO_WritePin(LED_LCD_GPIO_Port, LED_LCD_Pin, GPIO_PIN_SET);
		char rx_buff_byte[10] = { 0 };
		HAL_StatusTypeDef ret = HAL_UART_Receive(&huart1, rx_buff_byte, 10,
				1000);
		if (HAL_OK == ret) {
			char buf[10];
			sprintf(buf, "wrote: %s", rx_buff_byte);
			lcd_display_write_line(2, buf);
		}

		//HAL_UART_Receive(&huart1, NULL, 100, 100);
	}
}

uint8_t UART1_rxBuffer = { 0 };

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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

	lcd_display_init_task();

	HAL_TIM_Base_Start_IT(&htim7); // 2s heartbeat
	HAL_UART_Receive_IT(&huart1, &UART1_rxBuffer, 1);


	HAL_ADCEx_Calibration_Start(&hadc1); // MAGIE?

	//HAL_ADC_Start_IT(&hadc1);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcval, sizeof(adcval) / sizeof(adcval[0]) );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		HAL_GPIO_WritePin(LED_LCD_GPIO_Port, LED_LCD_Pin, GPIO_PIN_SET);
		while (1) {
			// DO NOTHING!
		}

		while (1) {
			if (HAL_GPIO_ReadPin(NH_enter_GPIO_Port, NH_enter_Pin)) {
				HAL_GPIO_WritePin(LED_RIGHT_1_GPIO_Port, LED_RIGHT_1_Pin,
						GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(LED_RIGHT_1_GPIO_Port, LED_RIGHT_1_Pin,
						GPIO_PIN_RESET);
			}
		}
		//loop_print_screen();

		//HAL_Delay(6000);
		HAL_GPIO_WritePin(LED_LCD_GPIO_Port, LED_LCD_Pin, GPIO_PIN_SET);
		while (1) {
			HAL_Delay(2000);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			HAL_GPIO_WritePin(LED_LCD_GPIO_Port, LED_LCD_Pin, GPIO_PIN_RESET);
			uint8_t tx_buff[] = "hallo";
			HAL_UART_Transmit(&huart1, tx_buff, sizeof(tx_buff), 1000);
			HAL_GPIO_WritePin(LED_LCD_GPIO_Port, LED_LCD_Pin, GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(LED_LCD_GPIO_Port, LED_LCD_Pin, GPIO_PIN_RESET);
		blink_and_text();
		HAL_Delay(2000);
		HAL_GPIO_WritePin(LED_LCD_GPIO_Port, LED_LCD_Pin, GPIO_PIN_SET);
		blink_and_text();
		HAL_Delay(2000);

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static uint8_t c_up = 0;
static uint8_t c_down = 0;
static uint8_t c_left = 0;
static uint8_t c_right = 0;
static uint8_t c_enter = 0;
static uint32_t exti_tick = 0;
#define EXTI_DISP_LINE 0

void send_uart(const char * json){
	char line[250];
	size_t len = sprintf(line, "%s\n", json);
	if(HAL_UART_Transmit(&huart1, (uint8_t *)line, len, 1000) == HAL_OK)
	{
		__NOP();
	}
}


void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	char buf[16];

	/* cheap DEBOUNCE 100ms */
	uint32_t ticks_now = HAL_GetTick();
	uint32_t ticks_elapsed = ticks_now - exti_tick;	// in ms
	if (ticks_elapsed < 100) {
		//sprintf(buf, "ret just: %lu ticks", ticks_elapsed);
		//lcd_display_write_line(1, buf);
		return;
	}
	exti_tick = ticks_now;

	HAL_GPIO_WritePin(LED_LEFT_1_GPIO_Port, LED_LEFT_1_Pin, GPIO_PIN_SET);

	switch (GPIO_Pin) {
	case NH_right_Pin:
		send_uart("{\"keypress\":\"right\"}");
		//sprintf(buf, "right: %03d", ++c_right);
		//lcd_display_write_line(EXTI_DISP_LINE, buf);
		break;
	case NH_left_Pin:
		send_uart("{\"keypress\":\"left\"}");
		//sprintf(buf, "left: %03d", ++c_left);
		//lcd_display_write_line(EXTI_DISP_LINE, buf);
		break;
	case NH_up_Pin:
		send_uart("{\"keypress\":\"up\"}");
		//sprintf(buf, "up: %03d", ++c_up);
		//lcd_display_write_line(EXTI_DISP_LINE, buf);
		break;
	case NH_down_Pin:
		send_uart("{\"keypress\":\"down\"}");
		//sprintf(buf, "down: %03d", ++c_down);
		lcd_display_write_line(EXTI_DISP_LINE, buf);
		break;
	case NH_enter_Pin:
		send_uart("{\"keypress\":\"enter\"}");
		//sprintf(buf, "enter: %03d", ++c_enter);
		//lcd_display_write_line(EXTI_DISP_LINE, buf);
		break;
	default:
		__NOP();
	}
	HAL_GPIO_WritePin(LED_LEFT_1_GPIO_Port, LED_LEFT_1_Pin, GPIO_PIN_RESET);

	// DEBUG
	//sprintf(buf, "t: %lu", ticks_elapsed);
	//lcd_display_write_line(1, buf);
}

void statham(char *json_bytes) {
	json_t mem[32];
	json_t const *json = json_create(json_bytes, mem, sizeof mem / sizeof *mem);
	if (!json) {
		return;
	}

	for (uint8_t line = 0; line < 4; line++) {
		char line_prop_name[6] = { 0 };
		sprintf(line_prop_name, "line%d", line);

		json_t const *lp = json_getProperty(json, line_prop_name);
		if (0 != lp && JSON_TEXT == json_getType(lp)) {
			char const *lineVal = json_getValue(lp);
				lcd_display_write_line(line, lineVal);
		}
	}

}

static uint8_t rx_buf[120] = { 0 };
uint8_t rx_buf_idx = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_GPIO_WritePin(LED_LEFT_3_GPIO_Port, LED_LEFT_3_Pin, GPIO_PIN_SET);
	rx_buf[rx_buf_idx++] = UART1_rxBuffer;

	if (rx_buf_idx == 120) {
		char buf[125];
		// overflow!!!
		sprintf(buf, "txt: %s", rx_buf);
		lcd_display_write_line(3, buf);
		memset(rx_buf, 0, sizeof(rx_buf));
		rx_buf_idx = 0;
	}

	if (UART1_rxBuffer == '\n') {
		rx_buf[rx_buf_idx] = '\0';	//TERM
		statham((char*) rx_buf);
		memset(rx_buf, 0, sizeof(rx_buf));
		rx_buf_idx = 0;
	}

	if (rx_buf_idx == 30) {
		__NOP();
	}

	HAL_GPIO_WritePin(LED_LEFT_3_GPIO_Port, LED_LEFT_3_Pin, GPIO_PIN_RESET);
	HAL_UART_Receive_IT(&huart1, &UART1_rxBuffer, 1); // get next
}

void heartbeat(){
	// quick and dirty 256x oversampling by gui
	char jbuf[240]={};
	sprintf(jbuf, "{\"0_ubat\":\"%d\",\"1_ibat\":\"%d\",\"2_1V5\":\"%d\",\"3_3V3\":\"%d\",\"4_inttemp\":\"%d\",\"5_intref\":\"%d\",\"heartbeat\":\"500ms_elapsed\"}",
			adcval[0],adcval[1],adcval[2],adcval[3],adcval[4],adcval[5]);
	send_uart(jbuf);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM7) {
		//send_uart("{\"heartbeat\":\"2s_elapsed\"}");
		heartbeat();
	}
	if (htim->Instance == TIM6) {
		// 500ms
		//heartbeat();
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	// from tim6
	//heartbeat();
}

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
	while (1) {
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
