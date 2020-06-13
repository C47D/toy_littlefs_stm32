/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "quadspi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Winbond_25Q.h"

#include "DBG.h"
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

static uint32_t read_ops = 0;
static uint32_t program_ops = 0;
static uint32_t erase_ops = 0;

#define TEST_LITTLEFS

#if defined TEST_LITTLEFS
#include "lfs.h"

// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;

// Read a region in a block
int user_provided_block_device_read(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size);

// Program a region in a block
// The block must be previously been erased
int user_provided_block_device_prog(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size);

// Erase a block
int user_provided_block_device_erase(const struct lfs_config *c, lfs_block_t block);

// Sync the state of the of the underlying block device
int user_provided_block_device_sync(const struct lfs_config *c);

typedef struct {
	uint8_t maj_version;
} user_ctx;

user_ctx my_ctx = {
	.maj_version = 0,
};

#define USER_CACHE_SIZE	4096
#define USER_BLOCK_SIZE 4096

// Note, memory must be 64-bit aligned
uint8_t my_read_buffer[USER_CACHE_SIZE] = {0};
uint8_t my_prog_buffer[USER_CACHE_SIZE] = {0};
uint8_t my_lookahead_buffer[USER_CACHE_SIZE] = {0};

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read  = user_provided_block_device_read,
    .prog  = user_provided_block_device_prog,
    .erase = user_provided_block_device_erase,
    .sync  = user_provided_block_device_sync,

    /* Block device configuration */

	/* Minimum size of a block read */
    .read_size = 16,
	/* Minimum size of a block program */
    .prog_size = 16,
	/* Size of an erasable block, must be a multiple of the
	 * read and program sizes */
    .block_size = USER_BLOCK_SIZE,
	// Number of erasable blocks on the device
    .block_count = 128,
	// Size of block caches.
	// Each cache buffers a portion of a block in RAM
    // The littlefs needs a read cache, a program cache, and one additional
    // cache per file. Larger caches can improve performance by storing more
    // data and reducing the number of disk accesses. Must be a multiple of
    // the read and program sizes, and a factor of the block size.
    .cache_size = 16,
    // Size of the lookahead buffer in bytes. A larger lookahead buffer
    // increases the number of blocks found during an allocation pass. The
    // lookahead buffer is stored as a compact bitmap, so each byte of RAM
    // can track 8 blocks. Must be a multiple of 8.
    .lookahead_size = 16,
    // Number of erase cycles before littlefs evicts metadata logs and moves
    // the metadata to another block. Suggested values are in the
    // range 100-1000, with large values having better performance at the cost
    // of less consistent wear distribution.
    //
    // Set to -1 to disable block-level wear-leveling.
    .block_cycles = 500,
    // Optional statically allocated read buffer. Must be cache_size.
    // By default lfs_malloc is used to allocate this buffer.
    .read_buffer = my_read_buffer,

    // Optional statically allocated program buffer. Must be cache_size.
    // By default lfs_malloc is used to allocate this buffer.
    .prog_buffer = my_prog_buffer,

    // Optional statically allocated lookahead buffer. Must be lookahead_size
    // and aligned to a 32-bit boundary. By default lfs_malloc is used to
    // allocate this buffer.
    .lookahead_buffer = my_lookahead_buffer,

	// context
	.context = &my_ctx
};

// Read a region in a block
int user_provided_block_device_read(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size)
{
	read_ops++;

	uint32_t read_address = block * c->block_size;
	uint32_t read_address_off = read_address + off;
	DBG_println("[READ] Block: %d, addr: %d, size: %d", (int) block, read_address_off, size);

	spi_read_data(read_address_off, buffer, size);

	return LFS_ERR_OK;
}

// Program a region in a block
// The block must be previously been erased
int user_provided_block_device_prog(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size)
{
	program_ops++;

	uint32_t prog_address = block * c->block_size;
	uint32_t prog_address_off = prog_address + off;

	DBG_println("[PROG] Block: %d, Address: %d, offset: %d, size: %d", block, prog_address, off, size);

	// Write Enable instruction before the device will accept any
	// Sector erase Intruction
	DBG_println("Enabling Writing...");
	spi_write_enable();
	HAL_Delay(10);
	spi_read_status_1();

	uint32_t spi_ret = spi_page_program(prog_address_off, (uint8_t *) buffer, size);

	DBG_println("BUSY...");
	while (spi_read_status_1() & STATUS_REGISTER_1_BUSY) {
		HAL_Delay(10);
	}
	DBG_println("No longer BUSY...");

	int ret = spi_ret == HAL_OK ? LFS_ERR_OK : LFS_ERR_CORRUPT;

	DBG_println("Operation result: %d", ret);

	return ret;
}

// Erase a block
// No veo alguna forma de optimizar esta funcion
int user_provided_block_device_erase(const struct lfs_config *c, lfs_block_t block)
{
	erase_ops++;

	// Convertimos de numero de bloque a direccion del bloque que littlefs quiere borrar
	uint32_t sector_address = block * c->block_size;

	DBG_println("[ERASE] Block: %d, addr = %d", (int) block, sector_address);

	// Write Enable instruction before the device will accept any
	// Sector erase Intruction
	DBG_println("Enabling writing...");
	spi_write_enable();
	HAL_Delay(10);
	spi_read_status_1();

	spi_erase_sector(sector_address);

	// After the /CS line going high, the self.timed Sector Erase instruction
	// will commence for a time duration of tSE (). While the Sector Erase
	// cycle is in progress, the Read Status Register instruction may still
	// be accessed for checking the status of the BUSY bit.
	// The BUSY bit is a 1 during the Sector Erase cycle and becomes a 0
	// when the cycle is finished and the device is ready to accept other
	// instructions again.
	// After the Sector Erase cycle has finished the WE bit in Status Register
	// is cleared to 0.
	//
	// tSE
	// For W25Q128FVxxIG 100ms
	// For W25Q128FVxxIQ or W25Q128FVxxIF 45ms

	// We have the W25Q128FVxxIQ so we poll the BUSY bit every 10ms
	while (spi_read_status_1() & STATUS_REGISTER_1_BUSY) {
		HAL_Delay(10);
	}

	return LFS_ERR_OK;
}

// Sync the state of the of the underlying block device
// See: https://github.com/ARMmbed/littlefs/issues/382
//
// Note the sync function is only needed if your NAND device has a cache
// that needs to be flushed to insure the data is stored in persistent storage.
// Otherwise sync can be a noop and return 0.
int user_provided_block_device_sync(const struct lfs_config *c)
{
	(void) c;

	// #382
	return LFS_ERR_OK;
}
#endif

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
  MX_QUADSPI_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  DBG_init(&huart3);
  DBG_clear_screen();
  DBG_println("QUADSPI Test - W25Q128FV");

#if 0

  spi_get_jedec_id();
  spi_read_status_1();
  spi_read_status_2();
  spi_read_status_3();

  DBG_println("Enabling write...");
  spi_write_enable();
  spi_read_status_1();
  DBG_println("Disabling write...");
  spi_write_disable();
  spi_read_status_1();

  uint8_t out_data[256] = {0};
  spi_read_data(0, out_data, sizeof out_data);
  DBG_hexdump(out_data, sizeof out_data, 0);

  DBG_println("Enabling write...");
  spi_write_enable();
  spi_read_status_1();

  uint8_t in_data[] = {
		  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
  };

  spi_page_program(0, in_data, sizeof in_data);

  spi_read_data(0, out_data, sizeof out_data);
  DBG_hexdump(out_data, sizeof out_data, 0);

  HAL_Delay(1000);

  spi_read_status_1();

  /* This write enable is necessary because the WEL bit
   * on the Status register 1 is set to 0 after the
   * page program instruction */
  spi_write_enable();
  spi_read_status_1();
  DBG_println("Erasing sector 0");
  spi_read_status_1();
  spi_erase_sector(0);
  spi_read_status_1();

  HAL_Delay(1000);

  spi_read_status_1();
  spi_write_enable();

  spi_read_data(0, out_data, sizeof out_data);
  DBG_hexdump(out_data, sizeof out_data, 0);

  DBG_println("Entering QPI mode...");
  DBG_println("Setting QE bit on Status register 2...");
  const uint8_t QE_BIT_POS = 1;
  uint8_t status2 = spi_read_status_2();
  DBG_println("Status register2 : %02Xh", status2);
  status2 |= (1 << QE_BIT_POS);
  DBG_println("Status register2 : %02Xh", status2);
  spi_write_status_2(status2);
  status2 = spi_read_status_2();
  DBG_println("Status register2 : %02Xh", status2);

  DBG_println("Sending ENTER_QPI_MODE instruction...");
  spi_enter_qpi_mode();

  DBG_println("Reading JEDEC ID...");
  qpi_read_jedec_id();

  DBG_println("Exiting QPI mode...");
  qpi_exit_qpi_mode();
#endif

#if defined TEST_LITTLEFS

#if 0
  DBG_println("Erasing chip...");
  spi_write_enable();
  spi_erase_chip();
  while (spi_read_status_1() & STATUS_REGISTER_1_BUSY) {
	  HAL_Delay(500);
  }
  DBG_println("Chip erased");
  spi_read_status_1();
#endif

  HAL_Delay(1000);

  // Mount the fs
  DBG_println("Mounting the FS...");
  int err = lfs_mount(&lfs, &cfg);

  // reformat if we can't mount the fs
  // this should be only happen on the first boot

  if (err) {
	  DBG_println("Error at mount: %d", err);
	  lfs_format(&lfs, &cfg);
	  err = lfs_mount(&lfs, &cfg);
	  DBG_println("Error at second mount: %d", err);
  }

#if 0
  // Static memory allocations, doesn't use the same api as the dinamic memory allocation
  // This is taken from #304
  const uint8_t *uploadFileName = "staticfile";
  lfs_file_t uploadFile;
  struct lfs_file_config uploadFileConfig;
  memset(&uploadFileConfig, 0, sizeof(struct lfs_file_config));

  // Optional statically allocated file buffer. Must be cache_size.
  uint8_t lfs_read_file_buffer[16];

  uploadFileConfig.buffer = lfs_read_file_buffer;  // use the static buffer

  // File attributes
  uploadFileConfig.attrs = NULL;
  // Number of custom attributes in the list
  uploadFileConfig.attr_count = 0;


  int res = lfs_file_opencfg(&lfs, &uploadFile, (char*) uploadFileName, LFS_O_RDONLY, &uploadFileConfig);
#endif

  // Read current count
  DBG_println("Opening file boot_count...");

  uint8_t boot_count = 0;
  err = lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
  if (err < 0) {
	  DBG_println("Error at opening the file: %d", err);
  }

  err = lfs_file_read(&lfs, &file, &boot_count, sizeof boot_count);
  if (err < 0) {
	  DBG_println("Error at reading the file: %d", err);
  } else {
	  DBG_println("Bytes read: %d", err);
	  DBG_println("Boot count: %d", boot_count);
  }

  // More file operations
  // lfs_file_truncate

  // Returns the position of the file
  lfs_soff_t file_pos = lfs_file_tell(&lfs, &file);
  if (file_pos < 0) {
	  DBG_println("Error while trying to get file position: %d", file_pos);
  } else {
	  DBG_println("Current file position: %d", file_pos);
  }

  // Update boot_count
  boot_count += 1;
  DBG_println("Boot count: %d", boot_count);
  err = lfs_file_rewind(&lfs, &file);
  if (err < 0) {
	  DBG_println("Error at rewinding the file: %d", err);
  } else {
	  DBG_println("Rewind ok");
  }

  file_pos = lfs_file_tell(&lfs, &file);
  if (file_pos < 0) {
	  DBG_println("Error while trying to get file position: %d", file_pos);
  } else {
	  DBG_println("Current file position: %d", file_pos);
  }

  err = lfs_file_write(&lfs, &file, &boot_count, sizeof boot_count);
  if (err < 0) {
	  DBG_println("Error at writing the file: %d", err);
  } else {
	  DBG_println("Bytes written into the file: %d", err);
  }

  lfs_soff_t file_size = lfs_file_size(&lfs, &file);
  if (file_size < 0) {
	  DBG_println("Error while trying to get size of file: %d", file_size);
  } else {
	  DBG_println("File size: %d", file_size);
  }

  // Remember the storage is not updated until the file is closed
  // successfully
  err = lfs_file_close(&lfs, &file);
  DBG_println("Closing file: %d", err);

  DBG_println("Boot count: %d", boot_count);

#if 0
  /* Directory operations */
  int dir_res = lfs_mkdir(&lfs, "test");
  if (dir_res < 0) {
	  DBG_println("Error while trying to create directory: %d", dir_res);
	  while (1);
  }

  lfs_dir_t test_dir;
  dir_res = lfs_dir_open(&lfs, &test_dir, "test");
  if (dir_res < 0) {
	  DBG_println("Error while trying to open directory: %d", dir_res);
	  while (1);
  }

  dir_res = lfs_dir_close(&lfs, &test_dir);
  if (dir_res < 0) {
	  DBG_println("Error while trying to open directory: %d", dir_res);
	  while (1);
  }
#endif

  // Release any resources we were using
  lfs_unmount(&lfs);

#endif

  DBG_println("Total read operations: %d", read_ops);
  DBG_println("Total program operations: %d", program_ops);
  DBG_println("Total erase operations: %d", erase_ops);

  asm("NOP");

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
