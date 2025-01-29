/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sirojuLIB_ST7789.h"
#include "sirojuLIB_ov7670_SCCB.h"
#include "image_bin.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "ff.h"
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
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;

DMA_HandleTypeDef hdma_memtomem_dma2_stream2;
DMA_HandleTypeDef hdma_memtomem_dma2_stream4;
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/*
 * SD CARD PINOUT
 * SD_DETECT	-> PD3
 * SDIO_CMD		-> PD2
 * SDIO_CK		-> PC12
 * SDIO_D0		-> PC8
 * SDIO_D1		-> PC9
 * SDIO_D2		-> PC10
 * SDIO_D3		-> PC11
 */

#define CAMERA_WIDTH  320
#define CAMERA_HEIGHT 240
#define LCD_WIDTH     240
#define LCD_HEIGHT    240
#define WIDTH  240
#define HEIGHT 240

#define LCD_PRINT(str, x, y) ST7789_printText(str, x, y, 0x0000, 0xffff, 1);

typedef enum {
	NONE, RED, GREEN, BLUE
}color_t;

_Bool bt_state = 0, lcd_show_flag = 0;
_Bool cam_frame_flag = 0, cam_show_mode = 1;
uint8_t camera_buffer[LCD_WIDTH*LCD_HEIGHT*2] __attribute__((aligned(32)));
uint8_t frame_color_mask[LCD_WIDTH*LCD_HEIGHT / 8] __attribute__((section(".ccmram")));
//__attribute__((section(".ccmram")))
uint8_t fps, fps_count = 0;
uint32_t timer1, fps_tick;
char lcd_str_buffer[32];

uint8_t frame_x0, frame_y0, frame_x1, frame_y1;

color_t filter_mode = RED;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

FATFS fs;         // FatFs work area
FIL fil;          // File object
FRESULT res;      // FatFs return code
char read_data[100]; // Buffer to store the read data
DWORD free_clust;  // Free clusters count
DWORD total_clust; // Total clusters count

#if 0
void sd_check (){
    // Mount the SD card
    res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        // Handle mounting error
        printf("Mounting SD card failed\n");
        while(1);
    }

    // Get the free space and total size of the SD card
    res = f_getfree("", &free_clust, &fs);
    if (res == FR_OK) {
        // Get the bytes per sector
        bytes_per_sector = fs.csize * 512;

        // Get the total clusters
        total_clust = fs.n_fatent - 2;  // Reserved clusters count

        // Calculate total size in bytes
        DWORD total_size = total_clust * bytes_per_sector;

        // Calculate free space in bytes
        DWORD free_size = free_clust * bytes_per_sector;

        // Format the information to display
        sprintf(info, "Total size: %lu bytes\nFree size: %lu bytes\n", total_size, free_size);

        // Print the SD card size information (via UART or any method)
        printf("%s", info);
    } else {
        // Handle error in getting free space
        printf("Error retrieving SD card information\n");
    }
}
#endif

void test_sd (){
	// Mount the SD card
	res = f_mount(&fs, "", 1);
	if (res != FR_OK) {
		// Handle mounting error
		LCD_PRINT ("mount failed", 5, 20);
		return;
	}
	LCD_PRINT ("mount success", 5, 20);

	// Open a file for writing
	res = f_open(&fil, "hello1.txt", FA_CREATE_ALWAYS | FA_WRITE);
	if (res == FR_OK) {
		LCD_PRINT ("success open file", 5, 35);
		char write_data[] = "Hello, STM32 SDIO!";
		UINT bytes_written;
		f_write(&fil, write_data, sizeof(write_data), &bytes_written);
		f_close(&fil);
	}
}

void test_sd_read_txt (){
	// Open the file for reading
	res = f_open(&fil, "hello.txt", FA_READ);
	if (res == FR_OK) {
		// Read data from the file
		UINT bytes_read;
		res = f_read(&fil, read_data, sizeof(read_data) - 1, &bytes_read);
		if (res == FR_OK) {
			// Null-terminate the string to make it safe for printing
			read_data[bytes_read] = '\0';
			// Do something with the data, e.g., print to console
//			sprintf (lcd_str_buffer, "Read data: %s", read_data);
//			ST7789_printText (lcd_str_buffer, 5, 5, 0x0000, 0xffff, 1);
		} else {
			// Handle read error
		}

		// Close the file after reading
		f_close(&fil);
	} else {
		// Handle file open error
	}
}
/************************************************************************************************************/

float get_fps (){
	float fps = 1000000.0 / (float)TIM2->CNT;
	TIM2->CNT = 0;
	return fps;
}

_Bool color_filter (color_t color, uint16_t pixel) {
	_Bool result = 0;
	uint8_t r = pixel >> 11;
	uint8_t g = (pixel >> 5) & 0x3f;
	uint8_t b = pixel & 0x1f;
	r = r << 3;
	g = g << 2;
	b = b << 3;

	if (color == RED){
    	if (r > (g + 25) && r > (b + 25) && r > 50) result = 1;
	}
	else if (color == GREEN){
    	if (g > (r + 100) && g > (b + 100) && g > 50) result = 1;
	}
	else if (color == BLUE){
    	if (b > (r + 0) && b > (g + 0) && b > 10) result = 1;
	}
	return result;
}

void cam_filter_color (color_t filter){
	uint8_t x0 = 255;
	uint8_t y0 = 255;
	uint8_t x1 = 0;
	uint8_t y1 = 0;
	for (int y = 0; y < HEIGHT-3; y+=3) {
		for (int x = 0; x < WIDTH-3; x+=3) {
			int index;
			uint16_t pixel_cam[9];
			_Bool result_filter = 1;
			for (uint8_t n = 0; n < 9; n++){
				index = ((y + n/3) * WIDTH + x + (n%3)) * 2;
				pixel_cam[n] = camera_buffer[index + 1] << 8 | camera_buffer[index];
				if (color_filter (filter, pixel_cam[n])){
					frame_color_mask[index/16] |= (1 << ((index/2) % 8));
				}
				else{
					frame_color_mask[index/16] &= ~(1 << ((index/2) % 8));
					result_filter = 0;
				}
//				result_filter = result_filter && color_filter (filter, pixel_cam[n]);

			}
			if (result_filter){
				if (x0 > x) x0 = x;
				if (y0 > y) y0 = y;
				if (x1 < x) x1 = x;
				if (y1 < y) y1 = y;

			}
		}
	}
	frame_x0 = x0;
	frame_y0 = y0;
	frame_x1 = x1;
	frame_y1 = y1;
}

void show_color_filter (color_t filter) {
	for (int y = 0; y < HEIGHT; y++) {
		for (int x = 0; x < WIDTH; x++) {
			int index = (y * WIDTH + x) * 2;
			uint16_t pixel_cam = camera_buffer[index + 1] << 8 | camera_buffer[index];
			if (color_filter (filter, pixel_cam)){
	        	ST7789_SendData (0xff);
	        	ST7789_SendData (0xff);
			}
			else{
	        	ST7789_SendData (0x00);
	        	ST7789_SendData (0x00);
			}
		}
	}
}

void lcd_show (uint8_t *ptr){
	for (int y = 0; y < HEIGHT; y++) {
		for (int x = 0; x < WIDTH; x++) {
			int index = (y * WIDTH + x) * 2;
			if (((x == frame_x0 || x == frame_x1) && y >= frame_y0 && y <= frame_y1) ||
				((y == frame_y0 || y == frame_y1) && x >= frame_x0 && x <= frame_x1)){
//				ptr[index + 1] = 0xf8;
//				ptr[index] = 0x00;
	        	ST7789_SendData (0xf8);
	        	ST7789_SendData (0x00);
			}
			else{
				ST7789_SendData (ptr[index + 1]);
				ST7789_SendData (ptr[index]);
			}
		}
	}
}

void lcd_show_color_mask (uint8_t *ptr){
	for (int y = 0; y < HEIGHT; y++) {
		for (int x = 0; x < WIDTH; x++) {
			int index = (y * WIDTH + x);
			if ((frame_color_mask[index/8] >> (index % 8)) & 1){
	        	ST7789_SendData (0xff);
	        	ST7789_SendData (0xff);
			}
			else{
	        	ST7789_SendData (0x00);
	        	ST7789_SendData (0x00);
			}
		}
	}
}

void lcd_print_fps (){
	sprintf (lcd_str_buffer, "%.2ffps ", get_fps ());
	lcd_print_str_to_buffer (camera_buffer, lcd_str_buffer, 5, 5, 0xf800, 0, 1);
}

void SendImageUSB(uint8_t *data, uint32_t length) {
    uint32_t header = 0xA5A5A5A5;

    // Kirim header
    while (CDC_Transmit_FS((uint8_t *)&header, sizeof(header)) == USBD_BUSY);

    // Kirim data gambar
    while (length > 0) {
        uint32_t chunkSize = (length > 128) ? 128 : length; // Maksimum paket USB FS
        while (CDC_Transmit_FS(data, chunkSize) == USBD_BUSY); // Tunggu hingga endpoint USB siap
        data += chunkSize;
        length -= chunkSize;
    }
}

void send_image_with_frame() {
    uint32_t header = 0xA5A5A5A5;  // Header untuk sinkronisasi frame
    uint32_t offset = 0;

    // Kirim header
    while (CDC_Transmit_FS((uint8_t *)&header, sizeof(header)) == USBD_BUSY);

    for (int y = 0; y < HEIGHT; y++) {
    	int index;
    	if (y > frame_y0 && y < frame_y1){
            index = (y * WIDTH + frame_x0) * 2;  // Hitung indeks piksel
            camera_buffer[index + 1] = 0xF8; // Warna merah RGB565
            camera_buffer[index] = 0x00;
            index = (y * WIDTH + frame_x1) * 2;  // Hitung indeks piksel
            camera_buffer[index + 1] = 0xF8; // Warna merah RGB565
            camera_buffer[index] = 0x00;
    	}
    	else if (y == frame_y0 || y == frame_y1){
    		for (int x = frame_x0; x <= frame_x1; x++){
                index = (frame_y0 * WIDTH + x) * 2;  // Hitung indeks piksel
                camera_buffer[index + 1] = 0xF8; // Warna merah RGB565
                camera_buffer[index] = 0x00;
                index = (frame_y1 * WIDTH + x) * 2;  // Hitung indeks piksel
                camera_buffer[index + 1] = 0xF8; // Warna merah RGB565
                camera_buffer[index] = 0x00;
    		}
    	}
//        for (int x = 0; x < WIDTH; x++) {
//            if (((x == frame_x0 || x == frame_x1) && y >= frame_y0 && y <= frame_y1) ||
//                ((y == frame_y0 || y == frame_y1) && x >= frame_x0 && x <= frame_x1)) {
//                int index = (y * WIDTH + x) * 2;  // Hitung indeks piksel
//                camera_buffer[index + 1] = 0xF8; // Warna merah RGB565
//                camera_buffer[index] = 0x00;
//            }
//        }

        uint32_t length = WIDTH *2;
        while (length > 0){
        	uint32_t chunk_size = (length > 128) ? 128 : length;
            while (CDC_Transmit_FS(camera_buffer+offset, chunk_size) == USBD_BUSY);
            offset += chunk_size;
            length -= chunk_size;
        }
    }
}

uint32_t index_cam = 0;

void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi) {
}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi) {
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi) {
	lcd_print_fps ();
	if (cam_show_mode) {
		lcd_show (camera_buffer);
	}
	else{
		lcd_show_color_mask (camera_buffer);
	}
	lcd_show_flag = 1;
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
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_FSMC_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
//	HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	ST7789_Init(240, 240);
	HAL_GPIO_WritePin(CAM_EN_GPIO_Port, CAM_EN_Pin, GPIO_PIN_SET);

	ST7789_Fill (0xffff);
	ST7789_printImage(60, 60, 120, 120, dudububu, _LSB_FIRST);
	HAL_Delay(500);
	ST7789_Fill (0xffff);

//	LCD_PRINT ("Test SD Card", 5, 5);
//	test_sd ();
//	test_sd_read_txt ();

	ST7789_SetCursorPosition (0, 0, 239, 239);

	ov7670_config (&hi2c2, CAM_RST_GPIO_Port, CAM_RST_Pin);

	HAL_TIM_Base_Start(&htim2);

	HAL_DCMI_ConfigCrop(&hdcmi, 0, 0, LCD_WIDTH*2-1, LCD_HEIGHT*2-1);
	HAL_DCMI_EnableCrop(&hdcmi);
	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)&camera_buffer[0], LCD_WIDTH*LCD_HEIGHT/2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (!HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)){
		  if (!bt_state){
			  bt_state = 1;
			  if (cam_show_mode) cam_show_mode = 0;
			  else cam_show_mode = 1;
//			  filter_mode++;
//			  if (filter_mode > BLUE) filter_mode = RED;
//			  HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		  }
	  }
	  else{
		  if (bt_state) bt_state = 0;
	  }
//
	  if (lcd_show_flag){
		  cam_filter_color (filter_mode);
//		  send_image_with_frame ();
//		  SendImageUSB(camera_buffer, LCD_WIDTH*LCD_HEIGHT*2);
		  lcd_show_flag = 0;
	  }

	  if (HAL_GetTick() - timer1 > 500){
		  timer1 = HAL_GetTick();
		  HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
	  }



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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 839;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 167;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 100;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream2
  *   hdma_memtomem_dma2_stream4
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream2 on DMA2_Stream2 */
  hdma_memtomem_dma2_stream2.Instance = DMA2_Stream2;
  hdma_memtomem_dma2_stream2.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream2.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream2.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_memtomem_dma2_stream2.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream2.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream2.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_memtomem_dma2_stream2.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream2.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream2.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream2.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream2) != HAL_OK)
  {
    Error_Handler( );
  }

  /* Configure DMA request hdma_memtomem_dma2_stream4 on DMA2_Stream4 */
  hdma_memtomem_dma2_stream4.Instance = DMA2_Stream4;
  hdma_memtomem_dma2_stream4.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream4.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream4.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream4.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream4.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream4.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream4.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream4.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_memtomem_dma2_stream4.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream4.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream4.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream4.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream4) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_BUILTIN_Pin|SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_BL_Pin|LCD_RST_Pin|CAM_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_EN_Pin|CAM_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BL_Pin LCD_RST_Pin CAM_RST_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|LCD_RST_Pin|CAM_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_TE_Pin */
  GPIO_InitStruct.Pin = LCD_TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_EN_Pin CAM_EN_Pin */
  GPIO_InitStruct.Pin = LCD_EN_Pin|CAM_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU_INT_Pin */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_8;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 2;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 4;
  Timing.BusTurnAroundDuration = 1;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
