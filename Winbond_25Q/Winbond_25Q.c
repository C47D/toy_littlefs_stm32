/*
 * Winbond_25Q.c
 *
 *  Created on: 16/05/2020
 *      Author: d3bug
 */


#include "Winbond_25Q.h"

#include "quadspi.h"
#include "DBG.h"

typedef enum {
	SPI_INSTRUCTION_WRITE_ENABLE = 0x06,
	SPI_INSTRUCTION_WRITE_ENABLE_VOLATILE_STATUS_REGISTER = 0x50,
	SPI_INSTRUCTION_WRITE_DISABLE = 0x04,
	SPI_INSTRUCTION_READ_STATUS_1 = 0x05,
	SPI_INSTRUCTION_WRITE_STATUS_1 = 0x01,
	SPI_INSTRUCTION_READ_STATUS_2 = 0x35,
	SPI_INSTRUCTION_WRITE_STATUS_2 = 0x31,
	SPI_INSTRUCTION_READ_STATUS_3 = 0x15,
	SPI_INSTRUCTION_WRITE_STATUS_3 = 0x11,
	SPI_INSTRUCTION_CHIP_ERASE = 0xC7,
	SPI_INSTRUCTION_ERASE_PROGRAM_SUSPEND = 0x75,
	SPI_INSTRUCTION_ERASE_PROGRAM_RESUME = 0x7A,
	SPI_INSTRUCTION_POWER_DOWN = 0xB9,
	SPI_INSTRUCTION_RELEASE_POWERDOWN_ID = 0xAB,
	SPI_INSTRUCTION_MANUFACTURER_DEVICE_ID = 0x90,
	SPI_INSTRUCTION_READ_JEDEC_ID = 0x9F,
	SPI_INSTRUCTION_GLOBAL_BLOCK_LOCK = 0x7E,
	SPI_INSTRUCTION_GLOBAL_BLOCK_UNLOCK = 0x98,
	SPI_INSTRUCTION_ENTER_QPI_MODE = 0x38,
	SPI_INSTRUCTION_ENABLE_RESET = 0x66,
	SPI_INSTRUCTION_RESET_DEVICE = 0x99,
	SPI_INSTRUCTION_READ_UNIQUE_ID = 0x4B,
	SPI_INSTRUCTION_PAGE_PROGRAM = 0x02,
	SPI_INSTRUCTION_QUAD_PAGE_PROGRAM = 0x32,
	SPI_INSTRUCTION_SECTOR_ERASE_4KB = 0x20,
	SPI_INSTRUCTION_BLOCK_ERASE_32KB = 0x52,
	SPI_INSTRUCTION_BLOCK_ERASE_64KB = 0xD8,
	SPI_INSTRUCTION_READ_DATA = 0x03,
	SPI_INSTRUCTION_FAST_READ = 0x0B,
	SPI_INSTRUCTION_FAST_READ_DUAL_OUTPUT = 0x3B,
	SPI_INSTRUCTION_FAST_READ_QUAD_OUTPUT = 0x6B,
	SPI_INSTRUCTION_READ_SFDP_REGISTER = 0x5A,
	SPI_INSTRUCTION_ERASE_SECURITY_REGISTER = 0x44,
	SPI_INSTRUCTION_PROGRAM_SECURITY_REGISTER = 0x42,
	SPI_INSTRUCTION_READ_SECURITY_REGISTER = 0x48,
	SPI_INSTRUCTION_INDIVIDUAL_BLOCK_LOCK = 0x36,
	SPI_INSTRUCTION_INDIVIDUAL_BLOCK_UNLOCK = 0x39,
	SPI_INSTRUCTION_READ_BLOCK_LOCK = 0x3D,
	SPI_INSTRUCTION_FAST_READ_DUAL_IO = 0xBB,
	SPI_INSTRUCTION_MANUFACTURER_DEVICE_ID_DUAL_IO = 0x92,
	SPI_INSTRUCTION_SET_BURST_WITH_WRAP = 0x77,
	SPI_INSTRUCTION_FAST_READ_QUAD_IO = 0xEB,
	SPI_INSTRUCTION_WORD_READ_QUAD_IO = 0xE7,
	SPI_INSTRUCTION_OCTAL_WORD_READ_QUAD_IO = 0xE3,
	SPI_INSTRUCTION_MANUFACTURER_DEVICE_ID_QUAD_IO = 0x94,
} W25Q_spi_instruction;

typedef enum {
	QPI_INSTRUCTION_WRITE_ENABLE = 0x06,
	QPI_INSTRUCTION_WRITE_ENABLE_VOLATILE_STATUS_REGISTER = 0x50,
	QPI_INSTRUCTION_WRITE_DISABLE = 0x04,
	QPI_INSTRUCTION_READ_STATUS_1 = 0x05,
	QPI_INSTRUCTION_WRITE_STATUS_1 = 0x01,
	QPI_INSTRUCTION_READ_STATUS_2 = 0x35,
	QPI_INSTRUCTION_WRITE_STATUS_2 = 0x31,
	QPI_INSTRUCTION_READ_STATUS_3 = 0x15,
	QPI_INSTRUCTION_WRITE_STATUS_3 = 0x11,
	QPI_INSTRUCTION_CHIP_ERASE = 0xC7,
	QPI_INSTRUCTION_ERASE_PROGRAM_SUSPEND = 0x75,
	QPI_INSTRUCTION_ERASE_PROGRAM_RESUME = 0x7A,
	QPI_INSTRUCTION_POWER_DOWN = 0xB9,
	QPI_INSTRUCTION_SET_READ_PARAMETERS = 0xC0,
	QPI_INSTRUCTION_RELEASE_POWERDOWN_ID = 0xAB,
	QPI_INSTRUCTION_MANUFACTURER_DEVICE_ID = 0x90,
	QPI_INSTRUCTION_READ_JEDEC_ID = 0x9F,
	QPI_INSTRUCTION_GLOBAL_BLOCK_LOCK = 0x7E,
	QPI_INSTRUCTION_GLOBAL_BLOCK_UNLOCK = 0x98,
	QPI_INSTRUCTION_EXIT_QPI_MODE = 0xFF,
	QPI_INSTRUCTION_ENABLE_RESET = 0x66,
	QPI_INSTRUCTION_RESET_DEVICE = 0x99,
	QPI_INSTRUCTION_PAGE_PROGRAM = 0x02,
	QPI_INSTRUCTION_SECTOR_ERASE_4KB = 0x20,
	QPI_INSTRUCTION_BLOCK_ERASE_32KB = 0x52,
	QPI_INSTRUCTION_BLOCK_ERASE_64KB = 0xD8,
	QPI_INSTRUCTION_FAST_READ = 0x0B,
	QPI_INSTRUCTION_BURST_READ_WITH_WRAP = 0x0C,
	QPI_INSTRUCTION_FAST_READ_QUAD_IO = 0xEB,
	QPI_INSTRUCTION_INDIVIDUAL_BLOCK_LOCK = 0x36,
	QPI_INSTRUCTION_INDIVIDUAL_BLOCK_UNLOCK = 0x39,
	QPI_INSTRUCTION_READ_BLOCK_LOCK = 0x3D,
} W25Q_qpi_instruction;

/* set WEL to 1 (SR1) */
uint32_t spi_write_enable(void)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_WRITE_ENABLE,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_NONE,
		.NbData = 0,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
		return rcv_err;
	}

	return rcv_err;
}

uint32_t spi_write_enable_volatile_status_regiter(void)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_WRITE_ENABLE_VOLATILE_STATUS_REGISTER,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_NONE,
		.NbData = 0,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
		return rcv_err;
	}

	return rcv_err;
}

uint32_t spi_write_disable(void)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_WRITE_DISABLE,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_NONE,
		.NbData = 0,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
		return rcv_err;
	}

	return rcv_err;
}

uint32_t spi_get_jedec_id(void)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	uint8_t ID_REG[3] = {0};

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_READ_JEDEC_ID,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_1_LINE,
		.NbData = sizeof ID_REG,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
		return rcv_err;
	}

	rcv_err = HAL_QSPI_Receive(&hqspi, ID_REG, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to read memory JEDEC ID [%d]", rcv_err);
		return rcv_err;
	} else {
		DBG_println("JEDEC ID: %02X %02X %02X", ID_REG[0], ID_REG[1], ID_REG[2]);
	}

	return rcv_err;
}

uint32_t spi_read_data(uint32_t addr, uint8_t *buffer, size_t buffer_sz)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_READ_DATA,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_1_LINE,
		.AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = addr,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_1_LINE,
		.NbData = buffer_sz,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
		return rcv_err;
	}

	/* TODO: Calculate the timeout based on the spi clock rate */
	uint32_t rcv_timeout = 1000 * 5;
	rcv_err = HAL_QSPI_Receive(&hqspi, buffer, rcv_timeout);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to read data[%d]", rcv_err);
		return rcv_err;
	}

	return rcv_err;
}

uint32_t spi_fast_read_data(uint32_t addr, uint8_t *buffer, size_t buffer_sz)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_FAST_READ,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_1_LINE,
		.AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = addr,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 8,
		/* data phase */
		.DataMode = QSPI_DATA_1_LINE,
		.NbData = buffer_sz,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
		return rcv_err;
	}

	/* TODO: Calculate the timeout based on the spi clock rate
	 * and amount of requested data. */
	uint32_t rcv_timeout = 1000 * 5;
	rcv_err = HAL_QSPI_Receive(&hqspi, buffer, rcv_timeout);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to read data[%d]", rcv_err);
		return rcv_err;
	}

	return rcv_err;
}

uint32_t spi_fast_read_data_dual(uint32_t addr, uint8_t *buffer, size_t buffer_sz)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_FAST_READ_DUAL_OUTPUT,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_1_LINE,
		.AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = addr,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 8,
		/* data phase */
		.DataMode = QSPI_DATA_2_LINES,
		.NbData = buffer_sz,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
		return rcv_err;
	}

	/* TODO: Calculate the timeout based on the spi clock rate
	 * and amount of requested data. */
	uint32_t rcv_timeout = 1000 * 5;
	rcv_err = HAL_QSPI_Receive(&hqspi, buffer, rcv_timeout);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to read data[%d]", rcv_err);
		return rcv_err;
	}

	return rcv_err;
}

uint32_t spi_fast_read_data_dual_io(uint32_t addr, uint8_t *buffer, size_t buffer_sz)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_FAST_READ_DUAL_IO,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_2_LINES,
		.AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = addr,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 8,
		/* data phase */
		.DataMode = QSPI_DATA_2_LINES,
		.NbData = buffer_sz,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
		return rcv_err;
	}

	/* TODO: Calculate the timeout based on the spi clock rate
	 * and amount of requested data. */
	uint32_t rcv_timeout = 1000 * 5;
	rcv_err = HAL_QSPI_Receive(&hqspi, buffer, rcv_timeout);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to read data[%d]", rcv_err);
		return rcv_err;
	}

	return rcv_err;
}

uint32_t spi_fast_read_data_quad(uint32_t addr, uint8_t *buffer, size_t buffer_sz)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_FAST_READ_QUAD_OUTPUT,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_1_LINE,
		.AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = addr,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 8,
		/* data phase */
		.DataMode = QSPI_DATA_4_LINES,
		.NbData = buffer_sz,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
		return rcv_err;
	}

	/* TODO: Calculate the timeout based on the spi clock rate
	 * and amount of requested data. */
	uint32_t rcv_timeout = 1000 * 5;
	rcv_err = HAL_QSPI_Receive(&hqspi, buffer, rcv_timeout);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to read data[%d]", rcv_err);
		return rcv_err;
	}

	return rcv_err;
}

uint32_t spi_fast_read_data_quad_io(uint32_t addr, uint8_t *buffer, size_t buffer_sz)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_FAST_READ_QUAD_IO,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_2_LINES,
		.AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = addr,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 8,
		/* data phase */
		.DataMode = QSPI_DATA_4_LINES,
		.NbData = buffer_sz,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
		return rcv_err;
	}

	/* TODO: Calculate the timeout based on the spi clock rate
	 * and amount of requested data. */
	uint32_t rcv_timeout = 1000 * 5;
	rcv_err = HAL_QSPI_Receive(&hqspi, buffer, rcv_timeout);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to read data[%d]", rcv_err);
		return rcv_err;
	}

	return rcv_err;
}

/* TODO: Handle program data bigger than a page */

/* NOTE: If an entire 256 byte page is to be programmed,
 * the last address byte (the 8 least significant address
 * bits) should be set to 0. If the last address byte is
 * not zero, and the number of clocks exceeds the
 * remaining page length, the addressing will wrap
 * to the beginning of the page. In some cases less than
 * 256 bytes (a partial page) can be programmed without
 * having any effect on other bytes within the same page.
 * One condition to perform a partial page program is
 * that the number of clocks cannot exceed the remaining
 * page length. If more than 256 bytes are sent to the
 * device the addressing will wrap to the beginning
 * of the page and overwrite previously sent data. */
uint32_t spi_page_program(uint32_t offset, const uint8_t *buffer, size_t buffer_sz)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	if (PAGE_SIZE < buffer_sz) {
		return rcv_err;
	}

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_PAGE_PROGRAM,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_1_LINE,
		.AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = offset,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_1_LINE,
		.NbData = buffer_sz,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
		return rcv_err;
	}

	/* TODO: Calculate the timeout based on the spi clock rate */
	uint32_t tx_timeout = 1000 * 5;
	rcv_err = HAL_QSPI_Transmit(&hqspi, buffer, tx_timeout);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to page program [%d]", rcv_err);
		return rcv_err;
	}

	return rcv_err;
}

uint32_t spi_erase_chip(void)
{
	HAL_StatusTypeDef rcv_err = HAL_ERROR;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_CHIP_ERASE,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_NONE,
		.NbData = 0,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
	}

	return rcv_err;
}

uint32_t spi_erase_sector(uint32_t addr)
{
	HAL_StatusTypeDef rcv_err = HAL_ERROR;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_SECTOR_ERASE_4KB,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_1_LINE,
		.AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = addr,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_NONE,
		.NbData = 0,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
	}

	return rcv_err;
}

/* Sets all the memory within a specified block (32kbytes) to
 * the erased state of all 1s (FFh). A write enable instruction
 * must be executed before the device will accept the Block Erase
 * instruction.
 *
 * Time it takes to erase a block of 32kb is tBE1.
 *
 * While the Block Erase cycle is in progress, the Read Status
 * Register instruction may still be accessed for checking the
 * status of the BUSY bit.
 *
 * After the Block Erase cycle has finished the Write Enable Latch
 * bit in the Status Register is cleared to 0. */
uint32_t spi_erase_block_32kb(uint32_t addr)
{
	HAL_StatusTypeDef rcv_err = HAL_ERROR;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_BLOCK_ERASE_32KB,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_1_LINE,
		.AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = addr,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_NONE,
		.NbData = 0,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
	}

	return rcv_err;
}

uint32_t spi_erase_block_64kb(uint32_t addr)
{
	HAL_StatusTypeDef rcv_err = HAL_ERROR;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_BLOCK_ERASE_64KB,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_1_LINE,
		.AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = addr,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_NONE,
		.NbData = 0,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
	}

	return rcv_err;
}

uint32_t spi_enter_qpi_mode(void)
{
	HAL_StatusTypeDef rcv_err = HAL_ERROR;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_ENTER_QPI_MODE,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_NONE,
		.NbData = 0,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
	}

	return rcv_err;
}

uint32_t qpi_exit_qpi_mode(void)
{
	HAL_StatusTypeDef rcv_err = HAL_ERROR;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_4_LINES,
		.Instruction = QPI_INSTRUCTION_EXIT_QPI_MODE,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_NONE,
		.NbData = 0,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
	}

	return rcv_err;
}

uint32_t qpi_read_jedec_id(void)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	uint8_t ID_REG[3] = {0};

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_4_LINES,
		.Instruction = QPI_INSTRUCTION_READ_JEDEC_ID,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_4_LINES,
		.NbData = sizeof ID_REG,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
		return rcv_err;
	}

	rcv_err = HAL_QSPI_Receive(&hqspi, ID_REG, 1000);

	if (HAL_OK != rcv_err) {
		DBG_println("Error while trying to read memory JEDEC ID [%d]", rcv_err);
		return rcv_err;
	} else {
		DBG_println("JEDEC ID: %02X %02X %02X", ID_REG[0], ID_REG[1], ID_REG[2]);
	}

	return rcv_err;
}

uint8_t spi_read_status_1(void)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	uint8_t status1[1] = {0};

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_READ_STATUS_1,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_1_LINE,
		.NbData = sizeof status1,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK == rcv_err) {
		rcv_err = HAL_QSPI_Receive(&hqspi, status1, 1000);

		if (HAL_OK != rcv_err) {
			DBG_println("Error while trying to read status1 register [%d]", rcv_err);
		} else {
			DBG_println("Status1: %02X", status1[0]);
#if 0
			DBG_println("SRP0: %d", status1[0] & (1 << 7) ? 1 : 0);
			DBG_println("SEC: %d", status1[0] & (1 << 6) ? 1 : 0);
			DBG_println("TB: %d", status1[0] & (1 << 5) ? 1 : 0);
			DBG_println("BP2: %d", status1[0] & (1 << 4) ? 1 : 0);
			DBG_println("BP1: %d", status1[0] & (1 << 3) ? 1 : 0);
			DBG_println("BP0: %d", status1[0] & (1 << 2) ? 1 : 0);
			DBG_println("WEL: %d", status1[0] & (1 << 1) ? 1 : 0);
			DBG_println("BUSY: %d", status1[0] & (1 << 0) ? 1 : 0);
#endif
		}
	} else {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
	}

	return status1[0];
}

uint8_t spi_write_status_1(uint8_t status1_data)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_WRITE_STATUS_1,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_1_LINE,
		.NbData = 1,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK == rcv_err) {
		rcv_err = HAL_QSPI_Transmit(&hqspi, &status1_data, 1000);

		if (HAL_OK != rcv_err) {
			DBG_println("Error while trying to write status1 register [%d]", rcv_err);
		}
	} else {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
	}

	return rcv_err;
}

uint8_t spi_read_status_2(void)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	uint8_t status1[1] = {0};

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_READ_STATUS_2,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_1_LINE,
		.NbData = sizeof status1,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK == rcv_err) {
		rcv_err = HAL_QSPI_Receive(&hqspi, status1, 1000);

		if (HAL_OK != rcv_err) {
			DBG_println("Error while trying to read status2 register [%d]", rcv_err);
		} else {
			DBG_println("Status2: %02X", status1[0]);
			DBG_println("SUS: %d", status1[0] & (1 << 7) ? 1 : 0);
			DBG_println("CMP: %d", status1[0] & (1 << 6) ? 1 : 0);
			DBG_println("LB3: %d", status1[0] & (1 << 5) ? 1 : 0);
			DBG_println("LB2: %d", status1[0] & (1 << 4) ? 1 : 0);
			DBG_println("LB1: %d", status1[0] & (1 << 3) ? 1 : 0);
			DBG_println("R: %d", status1[0] & (1 << 2) ? 1 : 0);
			DBG_println("QE: %d", status1[0] & (1 << 1) ? 1 : 0);
			DBG_println("SRP1: %d", status1[0] & (1 << 0) ? 1 : 0);
		}
	} else {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
	}

	return status1[0];
}

uint8_t spi_write_status_2(uint8_t status2_data)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_WRITE_STATUS_2,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_1_LINE,
		.NbData = 1,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK == rcv_err) {
		rcv_err = HAL_QSPI_Transmit(&hqspi, &status2_data, 1000);

		if (HAL_OK != rcv_err) {
			DBG_println("Error while trying to write status1 register [%d]", rcv_err);
		}
	} else {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
	}

	return rcv_err;
}

uint8_t spi_read_status_3(void)
{
	HAL_StatusTypeDef rcv_err = HAL_OK;

	uint8_t status1[1] = {0};

	QSPI_CommandTypeDef command = {
		/* instruction phase */
		.InstructionMode = QSPI_INSTRUCTION_1_LINE,
		.Instruction = SPI_INSTRUCTION_READ_STATUS_3,
		/* address phase */
		.AddressMode = QSPI_ADDRESS_NONE,
		.AddressSize = 0,
		.Address = 0,
		/* alternate phase */
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
		.AlternateBytesSize = 0,
		.AlternateBytes = 0,
		/* dummy phase */
		.DummyCycles = 0,
		/* data phase */
		.DataMode = QSPI_DATA_1_LINE,
		.NbData = sizeof status1,
		/* misc */
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD
	};

	rcv_err = HAL_QSPI_Command(&hqspi, &command, 1000);

	if (HAL_OK == rcv_err) {
		rcv_err = HAL_QSPI_Receive(&hqspi, status1, 1000);

		if (HAL_OK != rcv_err) {
			DBG_println("Error while trying to read status3 register [%d]", rcv_err);
		} else {
			DBG_println("Status3: %02X", status1[0]);
			DBG_println("HOLD/RST: %d", status1[0] & (1 << 7) ? 1 : 0);
			DBG_println("DRV1: %d", status1[0] & (1 << 6) ? 1 : 0);
			DBG_println("DRV0: %d", status1[0] & (1 << 5) ? 1 : 0);
			DBG_println("R: %d", status1[0] & (1 << 4) ? 1 : 0);
			DBG_println("R: %d", status1[0] & (1 << 3) ? 1 : 0);
			DBG_println("WPS: %d", status1[0] & (1 << 2) ? 1 : 0);
			DBG_println("R: %d", status1[0] & (1 << 1) ? 1 : 0);
			DBG_println("R: %d", status1[0] & (1 << 0) ? 1 : 0);
		}
	} else {
		DBG_println("Error while trying to send the QSPI command [%d]", rcv_err);
	}

	return status1[0];
}
